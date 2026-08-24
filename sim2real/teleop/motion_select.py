#!/usr/bin/env python3
"""Interactively select motions for a running motion reference server."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import zmq

from serve_motion_reference import (
    MOTION_SELECT_PROTOCOL,
    discover_motion_files,
    motion_display_name,
)


BANNER = """\
Motion Selector
  - type a number or motion name to select it
  - press Enter to select the previous choice again
  - 'list'   : show all motions
  - 'status' : show server playback state
  - 'q'      : quit this selector

Workflow on G1:
  Press A once after starting the onboard deploy process to enter policy control.
  Each selection then starts automatically when the policy is at its default
  reference. After a motion finishes, the policy returns to default and waits.
"""


def print_menu(options: list[str]) -> None:
    print("\n=== Available motions ===")
    width = len(str(len(options)))
    for index, name in enumerate(options, 1):
        print(f"  {str(index).rjust(width)}. {name}")
    print("=========================\n")


def resolve_choice(value: str, options: list[str]) -> str:
    choice = value.strip().removesuffix(".npz")
    if choice.isdigit():
        index = int(choice)
        if 1 <= index <= len(options):
            return options[index - 1]
        raise ValueError(f"index out of range: {index}")
    if choice in options:
        return choice
    matches = [name for name in options if choice.lower() in name.lower()]
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        raise ValueError(f"ambiguous choice: {matches}")
    raise ValueError(f"unknown motion: {value!r}")


def send_request(endpoint: str, payload: dict[str, Any], timeout_ms: int) -> dict[str, Any]:
    socket = zmq.Context.instance().socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER, 0)
    socket.connect(endpoint)
    try:
        socket.send_json({"protocol": MOTION_SELECT_PROTOCOL, **payload})
        if not socket.poll(timeout=max(1, int(timeout_ms)), flags=zmq.POLLIN):
            raise TimeoutError(f"motion server did not reply within {timeout_ms} ms")
        reply = socket.recv_json()
    finally:
        socket.close(0)
    if not isinstance(reply, dict):
        raise RuntimeError(f"invalid motion server reply: {reply!r}")
    return reply


def print_reply(reply: dict[str, Any]) -> None:
    status = "OK" if bool(reply.get("ok", False)) else "REJECTED"
    state = str(reply.get("state", "unknown"))
    motion = str(reply.get("motion", "-"))
    detail = str(reply.get("detail", ""))
    suffix = f": {detail}" if detail else ""
    print(f"[{status}] state={state}, motion={motion}{suffix}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--motion-root",
        type=Path,
        default=Path("config/g1/motions"),
    )
    parser.add_argument(
        "--connect-addr",
        default="tcp://127.0.0.1:28704",
    )
    parser.add_argument("--timeout-ms", type=int, default=1000)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = args.motion_root.expanduser().resolve()
    options = [motion_display_name(path, root) for path in discover_motion_files(root)]
    if not options:
        raise RuntimeError(f"no .npz motions found under {root}")

    print_menu(options)
    print(BANNER)
    try:
        print_reply(send_request(args.connect_addr, {"command": "status"}, args.timeout_ms))
    except Exception as exc:
        print(f"[WARN] initial status failed: {exc}")

    last_choice: str | None = None
    while True:
        try:
            value = input(
                f"Select motion [number/name | Enter:{last_choice or '-'} | list | status | q]: "
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
        if value.lower() in ("s", "status"):
            try:
                print_reply(
                    send_request(args.connect_addr, {"command": "status"}, args.timeout_ms)
                )
            except Exception as exc:
                print(f"[ERROR] {exc}")
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
            reply = send_request(
                args.connect_addr,
                {"command": "select", "motion": choice},
                args.timeout_ms,
            )
            print_reply(reply)
            if bool(reply.get("ok", False)):
                last_choice = choice
        except Exception as exc:
            print(f"[ERROR] {exc}")


if __name__ == "__main__":
    main()
