"""Role identifiers shared by the bridge and transport header."""


ROLE_STATION = 'station'
ROLE_DRONE = 'drone'

ROLE_TO_CODE = {
    ROLE_STATION: 0,
    ROLE_DRONE: 1,
}
CODE_TO_ROLE = {code: role for role, code in ROLE_TO_CODE.items()}

VALID_ROLES = frozenset(ROLE_TO_CODE)


def role_code(role):
    """Return the on-wire numeric code for a configured bridge role."""
    try:
        return ROLE_TO_CODE[role]
    except KeyError as error:
        choices = ', '.join(sorted(VALID_ROLES))
        raise ValueError(
            f'invalid bridge role {role!r}; expected one of: {choices}'
        ) from error
