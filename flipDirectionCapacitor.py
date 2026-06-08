                "STORE missed 5.000s target. "
                "deltaE={:.3f}J Vcap={:.3f}V IL={:.3f}A".format(
                    E_delta,
                    va,
                    IL
                )
            )

            store_deadline_reported = True


        elif mode == "EXTRACT" and E_delta <= -(E_target_action - EXTRACT_DONE_TOL_J):

            elapsed_s = time.ticks_diff(
                time.ticks_ms(),
                action_start_ms
            ) / 1000.0

            print(
                "EXTRACT done. deltaE={:.3f}J time={:.3f}s "
                "E={:.3f}J Vcap={:.3f}V".format(
                    E_delta,
                    elapsed_s,
                    E,
                    va
                )
            )

            enter_maintain_after_extract(va)


    # --------------------------------------------------------
    # Current Control
    # --------------------------------------------------------

    control_allowed = (not hard_stopped) and ((not trip) or mode == "SAFE_HOLD")

    if control_allowed:

        if mode == "STORE":

            # ------------------------------------------------
            # SAFE STORE controller
            #
            # The old code jumped directly to about PWM=25000,
            # which caused 2.5-3.2 A overcurrent. In the 0.2 A test,
            # PWM around 12.6k caused a sudden negative-current trip.
            # This version caps STORE PWM within the active safety limits and ramps
            # upward faster only inside the safe range.
            # ------------------------------------------------

            IL_filtered = (
                (1.0 - IL_FILTER_ALPHA) * IL_filtered
                + IL_FILTER_ALPHA * IL
            )

            err = I_target - IL_filtered
            raw_step = STORE_PWM_GAIN * err

            if raw_step >= 0:
                pwm_step = clamp(
                    raw_step,
                    0.0,
                    get_store_pwm_step_up(va)
                )
            else:
                pwm_step = clamp(
                    raw_step,
                    -STORE_PWM_MAX_STEP_DOWN,
                    0.0
                )

            duty_cmd = clamp(
                duty_cmd + pwm_step,
                MIN_PWM,
                get_store_pwm_hard_max(va)
            )

            duty = int(duty_cmd)


        elif mode == "MAINTAIN" or mode == "SAFE_HOLD" or mode == "V_HOLD":

            if mode == "V_HOLD":
                I_target = calculate_vhold_current(va)

            IL_filtered = (
                (1.0 - IL_FILTER_ALPHA) * IL_filtered
                + IL_FILTER_ALPHA * IL
            )

            err = I_target - IL_filtered
            raw_step = MAINTAIN_PWM_GAIN * err

            if raw_step >= 0:
                pwm_step = clamp(
                    raw_step,
                    0.0,
                    MAINTAIN_PWM_MAX_STEP_UP
                )
            else:
                pwm_step = clamp(
                    raw_step,
                    -MAINTAIN_PWM_MAX_STEP_DOWN,
                    0.0
                )

            # Critical protection: do not allow the hold duty to fall
            # into the low-duty reverse-current region. At higher Vcap,
            # the safe minimum PWM must also be higher.
            duty_cmd = clamp(
                duty_cmd + pwm_step,
                calculate_active_maintain_min_pwm(va),
                get_store_pwm_hard_max(va)
            )

            duty = int(duty_cmd)


        elif mode == "EXTRACT":

            # ------------------------------------------------
            # EXTRACT controller
            #
            # Duty is lowered gradually from the MAINTAIN point.
            # This avoids the positive-current jump and the later
            # negative-current overshoot seen in the previous result.
            # ------------------------------------------------

            IL_filtered = (
                (1.0 - IL_FILTER_ALPHA) * IL_filtered
                + IL_FILTER_ALPHA * IL
            )

            I_target = calculate_extract_target_current(E)

            err = I_target - IL_filtered

            pwm_step = clamp(
                EXTRACT_PWM_GAIN * err,
                -EXTRACT_PWM_MAX_STEP,
                EXTRACT_PWM_MAX_STEP
            )

            duty_cmd = clamp(
                duty_cmd + pwm_step,
                EXTRACT_MIN_PWM,
                MAX_PWM
            )

            duty = int(duty_cmd)


        last_pwm_applied = duty
        write_active_pwm(last_pwm_applied)


    # --------------------------------------------------------
    # Periodic Status Output
    # --------------------------------------------------------

    if time.ticks_diff(now_ms, last_status_print_ms) >= STATUS_PRINT_INTERVAL_MS:
        last_status_print_ms = now_ms
        print_status()

    count += 1
