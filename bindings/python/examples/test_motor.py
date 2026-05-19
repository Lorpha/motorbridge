#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

from motorbridge import Controller, Mode


def _parse_id(text: str) -> int:
    return int(text, 0)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Robstride motor communication test over dm-serial bridge"
    )
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--model", default="rs-01")
    parser.add_argument("--motor-id", default="127")
    parser.add_argument("--feedback-id", default="0xFD")
    parser.add_argument(
        "--test",
        choices=["ping", "enable-disable", "read-state", "pos-vel", "rotate", "mit-rotate"],
        default="ping",
    )
    parser.add_argument("--pos", type=float, default=0.0)
    parser.add_argument("--vel", type=float, default=0.0)
    parser.add_argument("--kp", type=float, default=5.0)
    parser.add_argument("--kd", type=float, default=0.5)
    parser.add_argument("--tau", type=float, default=0.0)
    parser.add_argument("--vlim", type=float, default=1.0)
    parser.add_argument("--loop", type=int, default=20)
    parser.add_argument("--dt-ms", type=int, default=50)
    args = parser.parse_args()

    motor_id = _parse_id(args.motor_id)
    feedback_id = _parse_id(args.feedback_id)

    print(
        f"connecting via dm-serial {args.serial_port}@{args.baud} "
        f"motor_id={motor_id} feedback_id={feedback_id} model={args.model}"
    )

    with Controller.from_robstride_dm_serial(args.serial_port, args.baud) as ctrl:
        motor = ctrl.add_robstride_motor(motor_id, feedback_id, args.model)
        try:
            if args.test == "ping":
                device_id, responder_id = motor.robstride_ping()
                print(f"ping ok: device_id={device_id} responder_id={responder_id}")
                state = motor.get_state()
                print(f"state: {state}")

            elif args.test == "enable-disable":
                print("enabling...")
                ctrl.enable_all()
                time.sleep(0.5)
                state = motor.get_state()
                print(f"after enable: {state}")
                print("disabling...")
                ctrl.disable_all()
                time.sleep(0.2)
                state = motor.get_state()
                print(f"after disable: {state}")

            elif args.test == "read-state":
                ctrl.enable_all()
                motor.ensure_mode(Mode.MIT, 1000)
                for i in range(args.loop):
                    motor.send_mit(0.0, 0.0, 0.0, 0.0, 0.0)
                    state = motor.get_state()
                    print(f"#{i} {state}")
                    if args.dt_ms > 0:
                        time.sleep(args.dt_ms / 1000.0)

            elif args.test == "pos-vel":
                ctrl.enable_all()
                motor.ensure_mode(Mode.POS_VEL, 1000)
                for i in range(args.loop):
                    motor.robstride_write_param_f32(0x7017, abs(args.vlim))
                    motor.robstride_write_param_f32(0x7016, args.pos)
                    state = motor.get_state()
                    print(f"#{i} target={args.pos:.3f} {state}")
                    if args.dt_ms > 0:
                        time.sleep(args.dt_ms / 1000.0)

            elif args.test == "rotate":
                ctrl.enable_all()
                motor.ensure_mode(Mode.POS_VEL, 1000)
                motor.robstride_write_param_f32(0x7017, abs(args.vlim))
                target = args.pos
                print(f"rotating to {target:.3f} rad, vlim={args.vlim}")
                for i in range(args.loop):
                    motor.robstride_write_param_f32(0x7016, target)
                    motor.request_feedback()
                    time.sleep(0.005)
                    ctrl.poll_feedback_once()
                    state = motor.get_state()
                    if state:
                        print(f"#{i} target={target:.3f} pos={state.pos:.3f} vel={state.vel:.3f} torq={state.torq:.3f}")
                    else:
                        print(f"#{i} target={target:.3f} state=None")
                    if args.dt_ms > 0:
                        time.sleep(args.dt_ms / 1000.0)

            elif args.test == "mit-rotate":
                ctrl.enable_all()
                motor.ensure_mode(Mode.MIT, 1000)
                print(f"MIT rotate: pos={args.pos} vel={args.vel} kp={args.kp} kd={args.kd} tau={args.tau}")
                for i in range(args.loop):
                    motor.send_mit(args.pos, args.vel, args.kp, args.kd, args.tau)
                    state = motor.get_state()
                    if state:
                        print(f"#{i} pos={state.pos:.3f} vel={state.vel:.3f} torq={state.torq:.3f}")
                    else:
                        print(f"#{i} state=None")
                    if args.dt_ms > 0:
                        time.sleep(args.dt_ms / 1000.0)

        finally:
            try:
                motor.disable()
            except Exception:
                pass
            motor.close()


if __name__ == "__main__":
    main()
