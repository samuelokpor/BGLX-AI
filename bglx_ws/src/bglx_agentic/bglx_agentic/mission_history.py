"""Persistent lightweight mission history for the BGLX prototype."""

import datetime
import json
import os
import secrets


MISSION_HISTORY_PATH = os.path.expanduser(
    '~/.bglx/mission_history.jsonl'
)


def utc_now_iso():
    """Current UTC timestamp in a compact ISO-8601 form."""

    return (
        datetime.datetime.now(
            datetime.timezone.utc
        )
        .isoformat(
            timespec='seconds'
        )
        .replace(
            '+00:00',
            'Z'
        )
    )


def new_mission_id():
    """Generate a human-readable, practically unique mission ID."""

    stamp = datetime.datetime.now().strftime(
        '%Y%m%d-%H%M%S'
    )

    suffix = secrets.token_hex(
        2
    ).upper()

    return (
        'M-%s-%s'
        % (
            stamp,
            suffix,
        )
    )


def append_mission_record(
    record,
    path=None
):
    """Append one completed/stopped mission record to JSONL."""

    if path is None:
        path = MISSION_HISTORY_PATH

    directory = os.path.dirname(
        path
    )

    if directory:

        os.makedirs(
            directory,
            exist_ok=True
        )

    line = json.dumps(
        record,
        sort_keys=True,
        ensure_ascii=False
    )

    with open(
        path,
        'a',
        encoding='utf-8'
    ) as f:

        f.write(
            line + '\n'
        )

        f.flush()
        os.fsync(
            f.fileno()
        )


def read_mission_history(
    limit=5,
    path=None
):
    """
    Return newest mission records first.

    Malformed individual lines are ignored rather than making the
    complete history unreadable.
    """

    if path is None:
        path = MISSION_HISTORY_PATH

    try:
        limit = int(
            limit
        )
    except Exception:
        limit = 5

    limit = max(
        1,
        min(
            limit,
            20
        )
    )

    if not os.path.exists(
        path
    ):
        return []

    records = []

    with open(
        path,
        'r',
        encoding='utf-8'
    ) as f:

        for raw in f:

            raw = raw.strip()

            if not raw:
                continue

            try:
                record = json.loads(
                    raw
                )
            except Exception:
                continue

            if isinstance(
                record,
                dict
            ):
                records.append(
                    record
                )

    return list(
        reversed(
            records[-limit:]
        )
    )


def format_mission_history(
    limit=5,
    path=None
):
    """Human/agent-readable recent mission history."""

    records = read_mission_history(
        limit=limit,
        path=path
    )

    if not records:

        return (
            'MISSION HISTORY: no recorded '
            'delivery missions yet.'
        )

    lines = [
        'Recent BGLX delivery missions:'
    ]

    for record in records:

        mission_id = record.get(
            'mission_id',
            'UNKNOWN'
        )

        status = record.get(
            'status',
            'UNKNOWN'
        )

        route = record.get(
            'route',
            []
        )

        if isinstance(
            route,
            list
        ):
            route_text = ' -> '.join(
                str(x)
                for x in route
            )
        else:
            route_text = str(
                route
            )

        duration = record.get(
            'duration_sec'
        )

        if isinstance(
            duration,
            (int, float)
        ):
            duration_text = (
                '%.1fs'
                % duration
            )
        else:
            duration_text = 'unknown duration'

        finished = record.get(
            'finished_at',
            'unknown time'
        )

        lines.append(
            '%s | %s | %s | %s | finished %s'
            % (
                mission_id,
                route_text,
                status,
                duration_text,
                finished,
            )
        )

    return '\n'.join(
        lines
    )
