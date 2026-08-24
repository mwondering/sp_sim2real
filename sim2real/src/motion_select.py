#!/usr/bin/env python3
"""Select a motion that the deploy policy loads and plays onboard."""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path
from typing import Sequence

from common.udp_latest import UDPLatestSender
from paths import SIM2REAL_ROOT
from runtime.motion_sources import discover_motion_files


DEFAULT_MOTION_ROOT = SIM2REAL_ROOT.parent / "motion"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 28562

BANNER = """\
Onboard Motion Selector
  - type a number or motion name to select it
  - press Enter to select the previous choice again
  - 'list'   : show all motions
  - 'reload' : rescan motion_root
  - 'q'      : quit this selector

The policy loads the selected NPZ from onboard storage and then plays it from
memory. A selection may be queued while the policy is transitioning or playing;
the latest selection starts after the policy returns to its default reference.
"""


def available_motion_names(motion_root: Path) -> list[str]:
    return ["default", *discover_motion_files(motion_root).keys()]


def print_menu(options: Sequence[str]) -> None:
    print("\n=== Available onboard motions ===")
    width = len(str(len(options)))
    for index, name in enumerate(options, 1):
        print(f"  {str(index).rjust(width)}. {name}")
    print("===================================\n")


def resolve_choice(value: str, options: Sequence[str]) -> str:
    choice = value.strip().removesuffix(".npz")
    if choice.isdigit():
        index = int(choice)
        if 1 <= index <= len(options):
            return str(options[index - 1])
        raise ValueError(f"index out of range: {index}")
    if choice in options:
        return choice
    matches = [name for name in options if choice.lower() in name.lower()]
    if len(matches) == 1:
        return str(matches[0])
    if len(matches) > 1:
        raise ValueError(f"ambiguous choice: {matches}")
    raise ValueError(f"unknown motion: {value!r}")


def send_motion(sender: UDPLatestSender, name: str, host: str, port: int) -> None:
    sequence = sender.send({"source": "onboard-motion", "motion": name})
    timestamp = time.strftime("%H:%M:%S")
    print(
        f"[{timestamp}] Sent '{name}' to local policy "
        f"at udp://{host}:{port} (seq={sequence})"
    )
    print("Check the deploy policy window for Loaded/Append or Reject status.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--motion-root",
        type=Path,
        default=Path(os.environ.get("G1_MOTION_ROOT", DEFAULT_MOTION_ROOT)),
        help=f"Directory searched recursively for NPZ files (default: {DEFAULT_MOTION_ROOT}).",
    )
    parser.add_argument(
        "--host",
        default=os.environ.get("G1_MOTION_SELECT_HOST", DEFAULT_HOST),
        help="Policy command host. Keep 127.0.0.1 for onboard playback.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("G1_MOTION_SELECT_PORT", DEFAULT_PORT)),
    )
    parser.add_argument(
        "--list-only",
        action="store_true",
        help="Print discovered motion names and exit without opening a UDP socket.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not 1 <= int(args.port) <= 65535:
        raise ValueError("--port must be in [1, 65535]")
    motion_root = args.motion_root.expanduser().resolve()

    def reload_options() -> list[str]:
        names = available_motion_names(motion_root)
        print(f"[MotionSelect] motion_root={motion_root}, files={len(names) - 1}")
        print_menu(names)
        return names

    options = reload_options()
    if args.list_only:
        return

    sender = UDPLatestSender(args.host, args.port)
    print(BANNER)
    last_choice: str | None = None
    try:
        while True:
            try:
                value = input(
                    f"Select motion [number/name | Enter:{last_choice or '-'} | "
                    "list | reload | q]: "
                ).strip()
            except (EOFError, KeyboardInterrupt):
                print("\nBye.")
                return

            if value.lower() in ("q", "quit", "exit"):
                print("Bye.")
                return
            if value.lower() in ("l", "list"):
                print_menu(options)
                continue
            if value.lower() in ("r", "reload"):
                options = reload_options()
                continue
            if not value:
                if last_choice is None:
                    print("Nothing to reselect yet.")
                    continue
                choice = last_choice
            else:
                try:
                    choice = resolve_choice(value, options)
                except ValueError as exc:
                    print(f"[WARN] {exc}")
                    continue

            try:
                send_motion(sender, choice, args.host, args.port)
                last_choice = choice
            except OSError as exc:
                print(f"[ERROR] send failed: {exc}")
    finally:
        sender.close()


if __name__ == "__main__":
    main()
