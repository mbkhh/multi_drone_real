"""Role identifiers shared by the bridge and transport header."""


ROLE_STATION = 'station'
ROLE_LEADER = 'leader'
ROLE_FOLLOWER = 'follower'

# ``drone`` was the name of the flight-tested point-to-point role.  Accept it
# as an input alias for ``leader`` and keep its on-wire value (1) assigned to
# the leader so existing station/leader frames remain compatible.
ROLE_DRONE = 'drone'
ROLE_ALIASES = {ROLE_DRONE: ROLE_LEADER}

ROLE_TO_CODE = {
    ROLE_STATION: 0,
    ROLE_LEADER: 1,
    ROLE_FOLLOWER: 2,
}
CODE_TO_ROLE = {code: role for role, code in ROLE_TO_CODE.items()}

VALID_ROLES = frozenset(ROLE_TO_CODE)
VALID_ROLE_INPUTS = VALID_ROLES | frozenset(ROLE_ALIASES)


def normalize_role(role):
    """Return the canonical role for a launch/configuration value."""
    value = str(role).strip().lower()
    value = ROLE_ALIASES.get(value, value)
    if value not in VALID_ROLES:
        choices = ', '.join(sorted(VALID_ROLE_INPUTS))
        raise ValueError(
            f'invalid bridge role {role!r}; expected one of: {choices}'
        )
    return value


def role_code(role):
    """Return the on-wire numeric code for a configured bridge role."""
    try:
        return ROLE_TO_CODE[normalize_role(role)]
    except KeyError as error:
        choices = ', '.join(sorted(VALID_ROLE_INPUTS))
        raise ValueError(
            f'invalid bridge role {role!r}; expected one of: {choices}'
        ) from error
