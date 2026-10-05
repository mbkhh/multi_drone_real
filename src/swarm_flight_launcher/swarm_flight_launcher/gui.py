"""Tk desktop interface for starting and observing the real flight stack."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from queue import Empty
import threading
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

from swarm_flight_launcher.backend import (
    COMPONENT_ORDER,
    DEFAULT_REMOTE_WORKSPACE,
    DEFAULT_ROS_SETUP,
    EventBridge,
    load_settings,
    LocalStationProcess,
    RemoteDroneSession,
    save_settings,
)


SETTINGS_PATH = Path.home() / '.config' / 'swarm_flight_launcher' / 'config.json'
LOG_ROOT = Path.home() / 'swarm_launcher_logs'
MAX_TERMINAL_LINES = 5000
DEFAULT_DRONE_HOSTS = {1: '10.112.89.180'}
DEFAULT_REMOTE_USERNAME = 'mbkh'


class DroneRow:
    """Tk variables and widgets associated with one drone."""

    def __init__(self, parent, row_number, drone_id, defaults, callbacks):
        self.drone_id = int(drone_id)
        self.enabled = tk.BooleanVar(value=defaults.get('enabled', True))
        self.host = tk.StringVar(
            value=defaults.get('host', DEFAULT_DRONE_HOSTS.get(drone_id, ''))
        )
        self.port = tk.StringVar(value=str(defaults.get('port', 22)))
        self.username = tk.StringVar(
            value=defaults.get('username', DEFAULT_REMOTE_USERNAME)
        )
        self.password = tk.StringVar(value='')
        self.workspace = tk.StringVar(
            value=defaults.get('workspace', DEFAULT_REMOTE_WORKSPACE)
        )
        components = defaults.get('components', {})
        self.components = {
            name: tk.BooleanVar(value=components.get(name, True))
            for name in COMPONENT_ORDER
        }
        self.connection = tk.StringVar(value='DISCONNECTED')
        self.statuses = {
            name: tk.StringVar(value='STOPPED')
            for name in COMPONENT_ORDER
        }

        ttk.Checkbutton(parent, variable=self.enabled).grid(
            row=row_number, column=0, padx=2
        )
        ttk.Label(parent, text=f'Drone {drone_id}').grid(
            row=row_number, column=1, padx=4, sticky='w'
        )
        ttk.Entry(parent, textvariable=self.host, width=16).grid(
            row=row_number, column=2, padx=2, sticky='ew'
        )
        ttk.Entry(parent, textvariable=self.port, width=5).grid(
            row=row_number, column=3, padx=2
        )
        ttk.Entry(parent, textvariable=self.username, width=10).grid(
            row=row_number, column=4, padx=2
        )
        ttk.Entry(
            parent,
            textvariable=self.password,
            width=11,
            show='•',
        ).grid(row=row_number, column=5, padx=2)
        ttk.Entry(parent, textvariable=self.workspace, width=23).grid(
            row=row_number, column=6, padx=2, sticky='ew'
        )
        for offset, name in enumerate(COMPONENT_ORDER, start=7):
            box = ttk.Checkbutton(
                parent,
                text=name.capitalize(),
                variable=self.components[name],
            )
            box.grid(row=row_number, column=offset, padx=3)

        ttk.Label(
            parent,
            textvariable=self.connection,
            width=20,
        ).grid(row=row_number, column=10, padx=4)
        ttk.Button(
            parent,
            text='Connect',
            command=lambda: callbacks['connect'](self),
        ).grid(row=row_number, column=11, padx=2)
        ttk.Button(
            parent,
            text='Start checked',
            command=lambda: callbacks['start'](self),
        ).grid(row=row_number, column=12, padx=2)
        ttk.Button(
            parent,
            text='Stop drone',
            command=lambda: callbacks['stop'](self),
        ).grid(row=row_number, column=13, padx=2)

    def snapshot(self):
        """Read Tk variables on the GUI thread."""
        try:
            port = int(self.port.get())
        except ValueError as error:
            raise ValueError(
                f'Drone {self.drone_id}: SSH port must be an integer'
            ) from error
        if not 1 <= port <= 65535:
            raise ValueError(
                f'Drone {self.drone_id}: SSH port must be from 1 to 65535'
            )
        return {
            'drone_id': self.drone_id,
            'enabled': bool(self.enabled.get()),
            'host': self.host.get().strip(),
            'port': port,
            'username': self.username.get().strip(),
            'password': self.password.get(),
            'workspace': self.workspace.get().strip(),
            'components': {
                name: bool(variable.get())
                for name, variable in self.components.items()
            },
        }

    def persistable(self):
        data = self.snapshot()
        data.pop('password', None)
        data.pop('drone_id', None)
        return data


class FlightLauncherApp:
    """One-window replacement for the ten manual terminals."""

    def __init__(self, root):
        self.root = root
        self.root.title('Multi-Drone Real Flight Launcher')
        self.root.geometry('1520x900')
        self.root.minsize(1180, 700)

        self.bridge = EventBridge()
        self.station = LocalStationProcess(self.bridge.emit)
        self.sessions = {
            drone_id: RemoteDroneSession(drone_id, self.bridge.emit)
            for drone_id in (1, 2, 3)
        }
        self.rows = {}
        self.terminals = {}
        self.tab_frames = {}
        self.status_variables = {'station': tk.StringVar(value='STOPPED')}
        self.log_handles = {}
        self.shutting_down = False
        self.run_directory = LOG_ROOT / datetime.now().strftime(
            '%Y%m%d_%H%M%S'
        )
        self.run_directory.mkdir(parents=True, exist_ok=True)

        try:
            self.loaded_settings = load_settings(SETTINGS_PATH)
        except Exception as error:
            self.loaded_settings = {}
            messagebox.showwarning(
                'Settings could not be loaded',
                f'{SETTINGS_PATH}\n\n{error}',
            )

        workspace_default = self._default_local_workspace()
        self.station_workspace = tk.StringVar(
            value=self.loaded_settings.get(
                'station_workspace',
                str(workspace_default),
            )
        )
        self.ros_setup = tk.StringVar(
            value=self.loaded_settings.get('ros_setup', DEFAULT_ROS_SETUP)
        )
        self.trust_new_hosts = tk.BooleanVar(
            value=bool(self.loaded_settings.get('trust_new_hosts', False))
        )
        self.footer = tk.StringVar(
            value=f'Persistent output: {self.run_directory}'
        )

        self._build_interface()
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)
        self.root.after(75, self._poll_events)

    @staticmethod
    def _default_local_workspace():
        current = Path.cwd()
        if (current / 'install' / 'setup.bash').exists():
            return current
        fallback = Path.home() / 'multi_drone_real'
        return fallback

    def _build_interface(self):
        outer = ttk.Frame(self.root, padding=8)
        outer.pack(fill='both', expand=True)

        toolbar = ttk.LabelFrame(outer, text='Laptop station', padding=6)
        toolbar.pack(fill='x')
        ttk.Label(toolbar, text='Workspace').grid(row=0, column=0, sticky='w')
        ttk.Entry(
            toolbar,
            textvariable=self.station_workspace,
            width=42,
        ).grid(row=0, column=1, padx=4, sticky='ew')
        ttk.Label(toolbar, text='ROS setup').grid(row=0, column=2, sticky='w')
        ttk.Entry(
            toolbar,
            textvariable=self.ros_setup,
            width=30,
        ).grid(row=0, column=3, padx=4, sticky='ew')
        ttk.Label(
            toolbar,
            textvariable=self.status_variables['station'],
            width=18,
        ).grid(row=0, column=4, padx=4)
        ttk.Button(
            toolbar,
            text='Start station',
            command=self.start_station,
        ).grid(row=0, column=5, padx=2)
        ttk.Button(
            toolbar,
            text='Stop station',
            command=self.stop_station,
        ).grid(row=0, column=6, padx=2)
        toolbar.columnconfigure(1, weight=1)
        toolbar.columnconfigure(3, weight=1)

        drones_frame = ttk.LabelFrame(
            outer,
            text='Remote drones (passwords are never saved)',
            padding=6,
        )
        drones_frame.pack(fill='x', pady=(7, 0))
        headings = (
            'Use', 'Drone', 'IP / host', 'Port', 'User', 'Password',
            'Remote workspace', 'Agent', 'Control', 'Vision',
            'SSH state', '', '', '',
        )
        for column, heading in enumerate(headings):
            ttk.Label(
                drones_frame,
                text=heading,
                font=('TkDefaultFont', 9, 'bold'),
            ).grid(row=0, column=column, padx=2, sticky='w')

        callbacks = {
            'connect': self.connect_drone,
            'start': self.start_drone,
            'stop': self.stop_drone,
        }
        drone_defaults = self.loaded_settings.get('drones', {})
        for row_number, drone_id in enumerate((1, 2, 3), start=1):
            defaults = drone_defaults.get(
                str(drone_id),
                drone_defaults.get(drone_id, {}),
            )
            row = DroneRow(
                drones_frame,
                row_number,
                drone_id,
                defaults,
                callbacks,
            )
            self.rows[drone_id] = row
            for component in COMPONENT_ORDER:
                self.status_variables[f'd{drone_id}/{component}'] = (
                    row.statuses[component]
                )
        drones_frame.columnconfigure(2, weight=1)
        drones_frame.columnconfigure(6, weight=1)

        action_bar = ttk.Frame(outer, padding=(0, 7, 0, 5))
        action_bar.pack(fill='x')
        ttk.Checkbutton(
            action_bar,
            text='Trust a new SSH host key on first connection',
            variable=self.trust_new_hosts,
        ).pack(side='left')
        ttk.Button(
            action_bar,
            text='Copy Drone 1 login to Drone 2/3',
            command=self.copy_first_login,
        ).pack(side='left', padx=8)
        ttk.Button(
            action_bar,
            text='Save non-secret settings',
            command=self.save_configuration,
        ).pack(side='left')
        ttk.Button(
            action_bar,
            text='STOP ALL MANAGED PROCESSES',
            command=self.stop_all,
        ).pack(side='right', padx=(6, 0))
        ttk.Button(
            action_bar,
            text='START SELECTED FLIGHT STACK',
            command=self.start_all,
        ).pack(side='right')

        status_grid = ttk.Frame(outer)
        status_grid.pack(fill='x', pady=(0, 5))
        ttk.Label(
            status_grid,
            text='Process states:',
            font=('TkDefaultFont', 9, 'bold'),
        ).pack(side='left')
        for drone_id in (1, 2, 3):
            for component in COMPONENT_ORDER:
                key = f'd{drone_id}/{component}'
                ttk.Label(
                    status_grid,
                    text=f' D{drone_id} {component}=',
                ).pack(side='left')
                ttk.Label(
                    status_grid,
                    textvariable=self.status_variables[key],
                ).pack(side='left')

        self.notebook = ttk.Notebook(outer)
        self.notebook.pack(fill='both', expand=True)
        self._add_terminal('station', 'Station')
        for drone_id in (1, 2, 3):
            for component in COMPONENT_ORDER:
                self._add_terminal(
                    f'd{drone_id}/{component}',
                    f'D{drone_id} {component.capitalize()}',
                )

        command_bar = ttk.Frame(outer, padding=(0, 6, 0, 0))
        command_bar.pack(fill='x')
        ttk.Label(command_bar, text='Station command:').pack(side='left')
        self.station_input = ttk.Entry(command_bar)
        self.station_input.pack(side='left', fill='x', expand=True, padx=5)
        self.station_input.bind('<Return>', lambda _event: self.send_station())
        ttk.Button(
            command_bar,
            text='Send',
            command=self.send_station,
        ).pack(side='left')
        ttk.Button(
            command_bar,
            text='Clear current view',
            command=self.clear_current_terminal,
        ).pack(side='left', padx=(5, 0))
        ttk.Label(outer, textvariable=self.footer).pack(
            fill='x', pady=(5, 0), anchor='w'
        )

    def _add_terminal(self, key, title):
        frame = ttk.Frame(self.notebook)
        terminal = scrolledtext.ScrolledText(
            frame,
            wrap='none',
            state='disabled',
            background='#101318',
            foreground='#e5e7eb',
            insertbackground='white',
            font=('DejaVu Sans Mono', 9),
        )
        terminal.pack(fill='both', expand=True)
        self.notebook.add(frame, text=title)
        self.terminals[key] = terminal
        self.tab_frames[key] = frame

    def _thread(self, name, target):
        threading.Thread(
            target=self._guarded_worker,
            args=(name, target),
            name=name,
            daemon=True,
        ).start()

    def _guarded_worker(self, key, target):
        output_key = key
        if key.startswith('d') and '/' not in key:
            output_key = f'{key}/agent'
        try:
            target()
        except Exception as error:
            self.bridge.emit('status', output_key, 'ERROR')
            self.bridge.emit(
                'output',
                output_key,
                f'[launcher] ERROR: {type(error).__name__}: {error}\n',
            )
            if key.startswith('d'):
                self.bridge.emit(
                    'connection',
                    key.split('/', 1)[0],
                    'ERROR - see output',
                )

    def connect_drone(self, row):
        try:
            snapshot = row.snapshot()
        except ValueError as error:
            messagebox.showerror('Invalid drone settings', str(error))
            return
        row.connection.set('CONNECTING')
        trust_new = bool(self.trust_new_hosts.get())

        def worker():
            self.sessions[row.drone_id].connect(
                host=snapshot['host'],
                username=snapshot['username'],
                password=snapshot['password'],
                port=snapshot['port'],
                trust_new_host=trust_new,
            )

        self._thread(
            f'd{row.drone_id}',
            worker,
        )

    def start_drone(self, row):
        try:
            snapshot = row.snapshot()
        except ValueError as error:
            messagebox.showerror('Invalid drone settings', str(error))
            return
        if not snapshot['host'] or not snapshot['username']:
            messagebox.showerror(
                'Missing SSH settings',
                f'Drone {row.drone_id} needs an IP/host and username.',
            )
            return
        selected = [
            component
            for component in COMPONENT_ORDER
            if snapshot['components'][component]
        ]
        if not selected:
            messagebox.showwarning(
                'Nothing selected',
                f'No process is checked for Drone {row.drone_id}.',
            )
            return

        row.connection.set('CONNECTING')
        trust_new = bool(self.trust_new_hosts.get())
        ros_setup = self.ros_setup.get().strip()

        def worker():
            session = self.sessions[row.drone_id]
            session.connect(
                snapshot['host'],
                snapshot['username'],
                snapshot['password'],
                snapshot['port'],
                trust_new,
            )
            for component in selected:
                session.start_component(
                    component,
                    snapshot['workspace'],
                    ros_setup,
                )
                # Let the XRCE agent establish its UDP socket before control.
                if component == 'agent':
                    time.sleep(0.7)

        self._thread(f'd{row.drone_id}', worker)

    def stop_drone(self, row):
        if not messagebox.askyesno(
            'Stop managed drone processes?',
            'Stopping the agent or controller while airborne transfers safety '
            'to PX4/RC failsafe behavior.\n\nOnly continue after landing and '
            'disarming, or when the pilot is prepared to take control.',
            icon='warning',
        ):
            return
        self._thread(
            f'd{row.drone_id}',
            self.sessions[row.drone_id].stop_all,
        )

    def start_station(self):
        try:
            self.station.start(
                self.station_workspace.get().strip(),
                self.ros_setup.get().strip(),
            )
        except Exception as error:
            messagebox.showerror('Station start failed', str(error))

    def stop_station(self):
        self._thread('station', self.station.stop)

    def start_all(self):
        self.start_station()
        for row in self.rows.values():
            if row.enabled.get():
                self.start_drone(row)

    def stop_all(self):
        if not messagebox.askyesno(
            'Stop all managed processes?',
            'This stops all three remote components and the station. Stopping '
            'flight communication while airborne is dangerous.\n\nOnly '
            'continue after landing/disarming or with RC/PX4 takeover ready.',
            icon='warning',
        ):
            return
        self._thread('station', self.station.stop)
        for drone_id, session in self.sessions.items():
            self._thread(f'd{drone_id}', session.stop_all)

    def send_station(self):
        command = self.station_input.get().strip()
        if not command:
            return
        try:
            self.station.send(command)
        except Exception as error:
            messagebox.showerror('Cannot send station command', str(error))
            return
        self.station_input.delete(0, 'end')

    def copy_first_login(self):
        source = self.rows[1]
        for drone_id in (2, 3):
            target = self.rows[drone_id]
            target.username.set(source.username.get())
            target.password.set(source.password.get())
            target.workspace.set(source.workspace.get())
            target.port.set(source.port.get())

    def configuration(self):
        return {
            'station_workspace': self.station_workspace.get().strip(),
            'ros_setup': self.ros_setup.get().strip(),
            'trust_new_hosts': bool(self.trust_new_hosts.get()),
            'drones': {
                str(drone_id): row.persistable()
                for drone_id, row in self.rows.items()
            },
        }

    def save_configuration(self):
        try:
            save_settings(SETTINGS_PATH, self.configuration())
        except Exception as error:
            messagebox.showerror('Settings save failed', str(error))
            return
        self.footer.set(
            f'Settings saved without passwords: {SETTINGS_PATH} | '
            f'Output: {self.run_directory}'
        )

    def _poll_events(self):
        try:
            while True:
                kind, key, value = self.bridge.queue.get_nowait()
                if kind == 'output':
                    self._append_output(key, str(value))
                elif kind == 'status':
                    variable = self.status_variables.get(key)
                    if variable is not None:
                        variable.set(str(value))
                    self._update_tab_title(key, str(value))
                elif kind == 'connection':
                    drone_id = int(key[1:])
                    self.rows[drone_id].connection.set(str(value))
                elif kind == 'shutdown_done':
                    self._final_destroy()
                    return
        except Empty:
            pass
        except Exception as error:
            self.footer.set(f'GUI event error: {error}')
        if self.root.winfo_exists():
            self.root.after(75, self._poll_events)

    def _update_tab_title(self, key, state):
        frame = self.tab_frames.get(key)
        if frame is None:
            return
        current = self.notebook.tab(frame, 'text').split(' [', 1)[0]
        short = state.split(' ', 1)[0]
        self.notebook.tab(frame, text=f'{current} [{short}]')

    def _append_output(self, key, value):
        terminal = self.terminals.get(key)
        if terminal is None:
            return
        terminal.configure(state='normal')
        terminal.insert('end', value)
        line_count = int(terminal.index('end-1c').split('.')[0])
        if line_count > MAX_TERMINAL_LINES:
            terminal.delete('1.0', f'{line_count - MAX_TERMINAL_LINES}.0')
        terminal.see('end')
        terminal.configure(state='disabled')

        handle = self.log_handles.get(key)
        if handle is None:
            path = self.run_directory / (key.replace('/', '_') + '.log')
            handle = path.open('a', encoding='utf-8', buffering=1)
            self.log_handles[key] = handle
        handle.write(value)

    def clear_current_terminal(self):
        selected = self.notebook.select()
        for key, frame in self.tab_frames.items():
            if str(frame) != selected:
                continue
            terminal = self.terminals[key]
            terminal.configure(state='normal')
            terminal.delete('1.0', 'end')
            terminal.configure(state='disabled')
            break

    def _has_managed_processes(self):
        if self.station.is_running():
            return True
        for session in self.sessions.values():
            for process in session.processes.values():
                if not process.channel.exit_status_ready():
                    return True
        return False

    def on_close(self):
        if self.shutting_down:
            return
        if self._has_managed_processes() and not messagebox.askyesno(
            'Exit and stop managed processes?',
            'Closing this GUI will stop every process that it started. Do not '
            'close while aircraft are airborne unless RC/PX4 takeover is ready.',
            icon='warning',
        ):
            return
        self.shutting_down = True
        self.footer.set('Stopping managed processes before exit...')

        def shutdown_worker():
            threads = []
            for session in self.sessions.values():
                thread = threading.Thread(target=session.stop_all, daemon=True)
                thread.start()
                threads.append(thread)
            self.station.stop()
            for thread in threads:
                thread.join(timeout=8.0)
            for session in self.sessions.values():
                session.close()
            self.bridge.emit('shutdown_done', 'gui', True)

        threading.Thread(
            target=shutdown_worker,
            name='launcher-shutdown',
            daemon=True,
        ).start()
        self.root.after(75, self._poll_events)

    def _final_destroy(self):
        for handle in self.log_handles.values():
            try:
                handle.close()
            except Exception:
                pass
        self.root.destroy()


def main(args=None):
    del args
    root = tk.Tk()
    FlightLauncherApp(root)
    root.mainloop()


if __name__ == '__main__':
    main()
