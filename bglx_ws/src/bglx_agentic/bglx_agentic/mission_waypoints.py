#!/usr/bin/env python3

"""Persistent named delivery-location registry for BGLX AI."""

import math
import os
import re

import yaml


CUSTOM_LOCATIONS_FILE = os.path.expanduser(
    '~/.bglx/delivery_locations.yaml'
)


BUILTIN_LOCATION_REGISTRY = {
    'HOME': {
        'x': 0.000,
        'y': 0.000,
        'type': 'depot',
        'display_name': 'Home Depot',
        'description': (
            'Primary BGLX mission start and return location.'
        ),
        'aliases': [
            'home',
            'base',
            'depot',
            'home depot',
        ],
    },

    'PICKUP_A': {
        'x': 5.514,
        'y': 0.098,
        'type': 'pickup',
        'display_name': 'Pickup A',
        'description': (
            'Primary parcel pickup/loading location in the '
            'current delivery test environment.'
        ),
        'aliases': [
            'pickup a',
            'pickup point a',
            'loading point a',
            'loading area a',
        ],
    },

    'DELIVERY_A': {
        'x': 4.969,
        'y': 5.492,
        'type': 'delivery',
        'display_name': 'Delivery A',
        'description': (
            'Primary parcel delivery/drop-off location in the '
            'current delivery test environment.'
        ),
        'aliases': [
            'delivery a',
            'delivery point a',
            'dropoff a',
            'drop off a',
            'drop-off a',
        ],
    },
}


VALID_LOCATION_TYPES = {
    'depot',
    'pickup',
    'delivery',
}


def _normalise_name(value):

    text = str(
        value
    ).strip().lower()

    text = re.sub(
        r'[_\-]+',
        ' ',
        text
    )

    text = re.sub(
        r'\s+',
        ' ',
        text
    )

    return text.strip()


def _canonical_name(value):

    text = str(
        value
    ).strip().upper()

    text = re.sub(
        r'[^A-Z0-9]+',
        '_',
        text
    )

    text = re.sub(
        r'_+',
        '_',
        text
    )

    return text.strip('_')


def _load_custom_locations():

    if not os.path.exists(
        CUSTOM_LOCATIONS_FILE
    ):
        return {}

    try:

        with open(
            CUSTOM_LOCATIONS_FILE,
            'r'
        ) as f:

            data = yaml.safe_load(
                f
            ) or {}

    except Exception:
        return {}

    if not isinstance(
        data,
        dict
    ):
        return {}

    result = {}

    for name, info in data.items():

        if not isinstance(
            info,
            dict
        ):
            continue

        canonical = _canonical_name(
            name
        )

        if not canonical:
            continue

        try:
            x = float(
                info['x']
            )
            y = float(
                info['y']
            )
        except (
            KeyError,
            TypeError,
            ValueError,
        ):
            continue

        location_type = str(
            info.get(
                'type',
                'delivery'
            )
        ).strip().lower()

        if location_type not in VALID_LOCATION_TYPES:
            continue

        aliases = info.get(
            'aliases',
            []
        )

        if not isinstance(
            aliases,
            list
        ):
            aliases = []

        result[canonical] = {
            'x': x,
            'y': y,
            'type': location_type,
            'display_name': str(
                info.get(
                    'display_name',
                    canonical.replace(
                        '_',
                        ' '
                    ).title()
                )
            ),
            'description': str(
                info.get(
                    'description',
                    'User-defined BGLX delivery location.'
                )
            ),
            'aliases': [
                str(a)
                for a in aliases
                if str(a).strip()
            ],
        }

    return result


def _build_registry():

    registry = {
        name: dict(info)
        for name, info
        in BUILTIN_LOCATION_REGISTRY.items()
    }

    custom = _load_custom_locations()

    # Protect the proven built-in HOME/PICKUP_A/DELIVERY_A geometry.
    for name, info in custom.items():

        if name in BUILTIN_LOCATION_REGISTRY:
            continue

        registry[name] = info

    return registry


LOCATION_REGISTRY = _build_registry()


LOCATIONS = {
    name: (
        float(info['x']),
        float(info['y']),
    )
    for name, info
    in LOCATION_REGISTRY.items()
}


def _alias_index():

    index = {}

    for canonical, info in LOCATION_REGISTRY.items():

        candidates = [
            canonical,
            info.get(
                'display_name',
                canonical
            ),
        ]

        candidates.extend(
            info.get(
                'aliases',
                []
            )
        )

        for alias in candidates:

            key = _normalise_name(
                alias
            )

            previous = index.get(
                key
            )

            if (
                previous is not None
                and previous != canonical
            ):
                raise ValueError(
                    "Duplicate mission location alias "
                    "'%s' used by %s and %s"
                    % (
                        alias,
                        previous,
                        canonical,
                    )
                )

            index[key] = canonical

    return index


_ALIAS_INDEX = _alias_index()


def reload_location_registry():
    """Reload persistent custom locations into this process."""

    global LOCATION_REGISTRY
    global LOCATIONS
    global _ALIAS_INDEX

    LOCATION_REGISTRY = _build_registry()

    LOCATIONS = {
        name: (
            float(info['x']),
            float(info['y']),
        )
        for name, info
        in LOCATION_REGISTRY.items()
    }

    _ALIAS_INDEX = _alias_index()


def resolve_location_name(name):

    key = _normalise_name(
        name
    )

    canonical = _ALIAS_INDEX.get(
        key
    )

    if canonical is None:

        available = ', '.join(
            sorted(
                LOCATION_REGISTRY.keys()
            )
        )

        raise ValueError(
            "Unknown mission location '%s'. "
            "Available canonical locations: %s"
            % (
                name,
                available,
            )
        )

    return canonical


def get_location_info(name):

    canonical = resolve_location_name(
        name
    )

    info = dict(
        LOCATION_REGISTRY[
            canonical
        ]
    )

    info['name'] = canonical

    return info


def get_location(name):

    canonical = resolve_location_name(
        name
    )

    return LOCATIONS[
        canonical
    ]


def list_location_names(
    location_type=None
):

    names = []

    for canonical, info in LOCATION_REGISTRY.items():

        if (
            location_type is not None
            and info.get('type') != location_type
        ):
            continue

        names.append(
            canonical
        )

    return sorted(
        names
    )


def format_location_registry():

    lines = []

    for canonical in sorted(
        LOCATION_REGISTRY.keys()
    ):

        info = LOCATION_REGISTRY[
            canonical
        ]

        aliases = ', '.join(
            info.get(
                'aliases',
                []
            )
        )

        lines.append(
            "%s [%s] - %s; aliases: %s"
            % (
                canonical,
                info.get(
                    'type',
                    'location'
                ),
                info.get(
                    'display_name',
                    canonical
                ),
                aliases or 'none',
            )
        )

    return '\n'.join(
        lines
    )


def save_custom_location(
    name,
    x,
    y,
    location_type,
    aliases=None,
    display_name=None,
):
    """
    Persist a NEW operator-defined mission location.

    Safety rules:
      - built-in locations cannot be overwritten;
      - existing custom locations cannot be overwritten accidentally;
      - canonical names, display names and aliases must not collide
        with any existing location;
      - validation happens before touching the persistent YAML file;
      - the final YAML replacement is atomic.
    """

    canonical = _canonical_name(
        name
    )

    if not canonical:

        raise ValueError(
            'Location name cannot be empty.'
        )

    if canonical in BUILTIN_LOCATION_REGISTRY:

        raise ValueError(
            "Built-in location '%s' cannot be overwritten."
            % canonical
        )

    location_type = str(
        location_type
    ).strip().lower()

    if location_type not in VALID_LOCATION_TYPES:

        raise ValueError(
            "Invalid location type '%s'. Valid types: %s"
            % (
                location_type,
                ', '.join(
                    sorted(
                        VALID_LOCATION_TYPES
                    )
                ),
            )
        )

    x = float(
        x
    )

    y = float(
        y
    )

    if aliases is None:
        aliases = []

    # Normalise / de-duplicate aliases while preserving
    # the operator-facing spelling.
    cleaned_aliases = []
    seen_aliases = set()

    for alias in aliases:

        alias = str(
            alias
        ).strip()

        if not alias:
            continue

        key = _normalise_name(
            alias
        )

        if key in seen_aliases:
            continue

        seen_aliases.add(
            key
        )

        cleaned_aliases.append(
            alias
        )

    aliases = cleaned_aliases

    if display_name:

        display_name = str(
            display_name
        ).strip()

    else:

        display_name = canonical.replace(
            '_',
            ' '
        ).title()

    custom = _load_custom_locations()

    # Recording a location is intentionally CREATE-ONLY.
    #
    # A separate explicit update operation can be added later.
    # This prevents:
    #
    #   "record this place as Building B"
    #
    # from silently moving an established delivery point.
    if canonical in custom:

        raise ValueError(
            "Custom location '%s' already exists. "
            "It was NOT overwritten."
            % canonical
        )

    # ------------------------------------------------------
    # TRANSACTIONAL ALIAS VALIDATION
    # ------------------------------------------------------
    #
    # Validate everything the new location will add to the
    # global alias namespace BEFORE writing the YAML file.
    # ------------------------------------------------------

    candidates = [
        canonical,
        display_name,
    ]

    candidates.extend(
        aliases
    )

    checked = set()

    for candidate in candidates:

        key = _normalise_name(
            candidate
        )

        if not key:
            continue

        # Duplicate names belonging to this same new location
        # are harmless, e.g.:
        #
        # BUILDING_B
        # Building B
        if key in checked:
            continue

        checked.add(
            key
        )

        owner = _ALIAS_INDEX.get(
            key
        )

        if owner is not None:

            raise ValueError(
                "Location name/alias '%s' conflicts with "
                "existing location %s. Nothing was saved."
                % (
                    candidate,
                    owner,
                )
            )

    custom[canonical] = {
        'x': x,
        'y': y,
        'type': location_type,
        'display_name': display_name,
        'description': (
            'User-defined BGLX delivery location.'
        ),
        'aliases': aliases,
    }

    directory = os.path.dirname(
        CUSTOM_LOCATIONS_FILE
    )

    os.makedirs(
        directory,
        exist_ok=True
    )

    # ------------------------------------------------------
    # ATOMIC PERSISTENCE
    # ------------------------------------------------------
    #
    # Write to a temporary file in the same directory and
    # replace the real registry only after the complete YAML
    # has been successfully written.
    # ------------------------------------------------------

    temp_path = (
        CUSTOM_LOCATIONS_FILE
        + '.tmp.%d'
        % os.getpid()
    )

    try:

        with open(
            temp_path,
            'w'
        ) as f:

            yaml.safe_dump(
                custom,
                f,
                sort_keys=True
            )

            f.flush()

            os.fsync(
                f.fileno()
            )

        os.replace(
            temp_path,
            CUSTOM_LOCATIONS_FILE
        )

    finally:

        if os.path.exists(
            temp_path
        ):

            os.remove(
                temp_path
            )

    reload_location_registry()

    return canonical


def _write_custom_locations_atomic(custom):
    """Atomically replace the persistent custom-location registry."""

    directory = os.path.dirname(
        CUSTOM_LOCATIONS_FILE
    )

    if directory:

        os.makedirs(
            directory,
            exist_ok=True
        )

    temp_path = (
        CUSTOM_LOCATIONS_FILE
        + '.tmp.%d'
        % os.getpid()
    )

    try:

        with open(
            temp_path,
            'w'
        ) as f:

            yaml.safe_dump(
                custom,
                f,
                sort_keys=True
            )

            f.flush()

            os.fsync(
                f.fileno()
            )

        os.replace(
            temp_path,
            CUSTOM_LOCATIONS_FILE
        )

    finally:

        if os.path.exists(
            temp_path
        ):

            os.remove(
                temp_path
            )


def update_custom_location(
    name,
    x=None,
    y=None,
    location_type=None,
    aliases=None,
    display_name=None,
):
    """
    Explicitly update an existing CUSTOM mission location.

    Unspecified fields retain their existing values.

    Built-in locations are immutable.
    """

    canonical = resolve_location_name(
        name
    )

    if canonical in BUILTIN_LOCATION_REGISTRY:

        raise ValueError(
            "Built-in location '%s' cannot be updated."
            % canonical
        )

    custom = _load_custom_locations()

    if canonical not in custom:

        raise ValueError(
            "Location '%s' is not a user-defined location."
            % canonical
        )

    current = dict(
        custom[canonical]
    )

    if x is not None:
        current['x'] = float(
            x
        )

    if y is not None:
        current['y'] = float(
            y
        )

    if location_type is not None:

        location_type = str(
            location_type
        ).strip().lower()

        if location_type not in VALID_LOCATION_TYPES:

            raise ValueError(
                "Invalid location type '%s'. Valid types: %s"
                % (
                    location_type,
                    ', '.join(
                        sorted(
                            VALID_LOCATION_TYPES
                        )
                    ),
                )
            )

        current['type'] = location_type

    if display_name is not None:

        display_name = str(
            display_name
        ).strip()

        if not display_name:

            raise ValueError(
                'Display name cannot be empty.'
            )

        current['display_name'] = display_name

    if aliases is not None:

        cleaned_aliases = []
        seen_aliases = set()

        for alias in aliases:

            alias = str(
                alias
            ).strip()

            if not alias:
                continue

            key = _normalise_name(
                alias
            )

            if key in seen_aliases:
                continue

            seen_aliases.add(
                key
            )

            cleaned_aliases.append(
                alias
            )

        current['aliases'] = cleaned_aliases

    # Validate the resulting alias namespace BEFORE writing.
    candidates = [
        canonical,
        current.get(
            'display_name',
            canonical
        ),
    ]

    candidates.extend(
        current.get(
            'aliases',
            []
        )
    )

    checked = set()

    for candidate in candidates:

        key = _normalise_name(
            candidate
        )

        if not key:
            continue

        if key in checked:
            continue

        checked.add(
            key
        )

        owner = _ALIAS_INDEX.get(
            key
        )

        # Existing aliases belonging to THIS location
        # are allowed during an explicit update.
        if (
            owner is not None
            and owner != canonical
        ):

            raise ValueError(
                "Location name/alias '%s' conflicts with "
                "existing location %s. Nothing was changed."
                % (
                    candidate,
                    owner,
                )
            )

    custom[canonical] = current

    _write_custom_locations_atomic(
        custom
    )

    reload_location_registry()

    return canonical


def delete_custom_location(name):
    """
    Delete an existing user-defined mission location.

    Built-in mission locations cannot be deleted.
    """

    canonical = resolve_location_name(
        name
    )

    if canonical in BUILTIN_LOCATION_REGISTRY:

        raise ValueError(
            "Built-in location '%s' cannot be deleted."
            % canonical
        )

    custom = _load_custom_locations()

    if canonical not in custom:

        raise ValueError(
            "Location '%s' is not a user-defined location."
            % canonical
        )

    del custom[canonical]

    _write_custom_locations_atomic(
        custom
    )

    reload_location_registry()

    return canonical

def arrival_pose(
    source_name,
    target_name
):

    sx, sy = get_location(
        source_name
    )

    tx, ty = get_location(
        target_name
    )

    yaw = math.atan2(
        ty - sy,
        tx - sx
    )

    return (
        tx,
        ty,
        yaw,
    )


def build_delivery_route(
    home_name,
    pickup_name,
    delivery_name
):

    home_name = resolve_location_name(
        home_name
    )

    pickup_name = resolve_location_name(
        pickup_name
    )

    delivery_name = resolve_location_name(
        delivery_name
    )

    return {
        'pickup': arrival_pose(
            home_name,
            pickup_name
        ),

        'delivery': arrival_pose(
            pickup_name,
            delivery_name
        ),

        'home': arrival_pose(
            delivery_name,
            home_name
        ),
    }
