use super::*;

macro_rules! add_motor_ffi {
    ($fn_name:ident, $ensure_fn:ident, $inner_variant:ident, $motor_ty:ty) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_name(
            controller: *mut MotorController,
            motor_id: u16,
            feedback_id: u16,
            model: *const c_char,
        ) -> *mut MotorHandle {
            if controller.is_null() {
                set_last_error("controller is null");
                return ptr::null_mut();
            }
            let model = match parse_cstr(model, "model") {
                Ok(v) => v,
                Err(e) => {
                    set_last_error(e);
                    return ptr::null_mut();
                }
            };
            let controller = unsafe { &mut *controller };

            if let ControllerInner::Mixed(ref core) = controller.inner {
                let bus = core.bus();
                let motor_result = <$motor_ty>::new(motor_id, feedback_id, &model, bus);
                match motor_result {
                    Ok(motor) => {
                        let motor = Arc::new(motor);
                        let device: Arc<dyn MotorDevice> = motor.clone();
                        match core.add_device(device) {
                            Ok(()) => Box::into_raw(Box::new(MotorHandle {
                                inner: MotorHandleInner::$inner_variant(motor),
                            })),
                            Err(e) => {
                                set_last_error(e.to_string());
                                ptr::null_mut()
                            }
                        }
                    }
                    Err(e) => {
                        set_last_error(e.to_string());
                        ptr::null_mut()
                    }
                }
            } else {
                match $ensure_fn(controller).and_then(|ctrl| {
                    ctrl.add_motor(motor_id, feedback_id, &model)
                        .map_err(|e| e.to_string())
                }) {
                    Ok(motor) => Box::into_raw(Box::new(MotorHandle {
                        inner: MotorHandleInner::$inner_variant(motor),
                    })),
                    Err(e) => {
                        set_last_error(e);
                        ptr::null_mut()
                    }
                }
            }
        }
    };
}

add_motor_ffi!(
    motor_controller_add_damiao_motor,
    ensure_damiao_controller,
    Damiao,
    DamiaoMotor
);
add_motor_ffi!(
    motor_controller_add_hexfellow_motor,
    ensure_hexfellow_controller,
    Hexfellow,
    HexfellowMotor
);
add_motor_ffi!(
    motor_controller_add_robstride_motor,
    ensure_robstride_controller,
    Robstride,
    RobstrideMotor
);
add_motor_ffi!(
    motor_controller_add_myactuator_motor,
    ensure_myactuator_controller,
    MyActuator,
    MyActuatorMotor
);

// HightorqueMotor::new takes different args (no Result), handle separately.
#[unsafe(no_mangle)]
pub extern "C" fn motor_controller_add_hightorque_motor(
    controller: *mut MotorController,
    motor_id: u16,
    feedback_id: u16,
    model: *const c_char,
) -> *mut MotorHandle {
    if controller.is_null() {
        set_last_error("controller is null");
        return ptr::null_mut();
    }
    let model = match parse_cstr(model, "model") {
        Ok(v) => v,
        Err(e) => {
            set_last_error(e);
            return ptr::null_mut();
        }
    };
    let controller = unsafe { &mut *controller };

    if let ControllerInner::Mixed(ref core) = controller.inner {
        let bus = core.bus();
        let motor = Arc::new(HightorqueMotor::new(motor_id, feedback_id, &model, bus));
        let device: Arc<dyn MotorDevice> = motor.clone();
        match core.add_device(device) {
            Ok(()) => Box::into_raw(Box::new(MotorHandle {
                inner: MotorHandleInner::Hightorque(motor),
            })),
            Err(e) => {
                set_last_error(e.to_string());
                ptr::null_mut()
            }
        }
    } else {
        match ensure_hightorque_controller(controller).and_then(|ctrl| {
            ctrl.add_motor(motor_id, feedback_id, &model)
                .map_err(|e| e.to_string())
        }) {
            Ok(motor) => Box::into_raw(Box::new(MotorHandle {
                inner: MotorHandleInner::Hightorque(motor),
            })),
            Err(e) => {
                set_last_error(e);
                ptr::null_mut()
            }
        }
    }
}
