# swarm_station

## Relative yaw command

After the leader is armed in Offboard and has taken off, use the normal
`move` command with a yaw parameter:

```text
move yaw=20
move yaw=-20
```

The value is a relative angle in PX4/NED degrees. The station sends it to the
leader only; followers receive the leader's resulting measured orientation in
`/swarm/local_state` and rotate their formation around the leader. Use either
`x`/`y`/`z` movement or `yaw` in one `move` command, not both.
