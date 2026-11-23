MODULE ModuleEGM
    ! EGM pose guidance for the FrezaEgm project (position only).
    ! Tool: tooldata toolPencil - TCP is the pencil tip (280 mm along flange Z).
    ! home = first point of data/trajectory.gcode.
    ! One EGM session per program start: the correction frame is created once.
    ! ASCII only: RAPID does not accept Cyrillic or UTF-8 punctuation.

    LOCAL CONST robtarget home := [[750, 0, 600], [0, 1, 0, 0], [0, 0, 0, 0], [9E9, 0, 0, 9E9, 9E9, 9E9]];

    ! Pencil TCP: 280 mm from the mounting flange along +Z (see toolPencil.rslib).
    ! If your pencil is longer/shorter, change 280 here.
    PERS tooldata toolPencil := [TRUE, [[0, 0, 280], [1, 0, 0, 0]], [0.05, [0, 0, 100], [1, 0, 0, 0], 0, 0, 0]];

    LOCAL VAR egmident egm_id;
    LOCAL VAR egm_minmax egm_condition := [-1, 1];
    LOCAL VAR pose corr_frame := [[0, 0, 0], [1, 0, 0, 0]];

    PROC main()
        MoveJ home, v100, fine, toolPencil;

        EGMGetId egm_id;

        EGMSetupUC ROB_1, egm_id, "default", "UCdevice" \Pose;

        ! Only X, Y, Z: orientation is not guided. With \Rx \Ry \Rz the controller
        ! keeps correcting the tool angles, the correction frame rotates and the
        ! robot drifts away from the trajectory.
        ! MaxSpeedDeviation removes the default low speed limit: without it the
        ! controller caps the speed and the feed setting has no effect.
        EGMActPose egm_id \Tool:=toolPencil \WObj:=wobj0, corr_frame, EGM_FRAME_BASE, corr_frame, EGM_FRAME_BASE
            \X:=egm_condition
            \Y:=egm_condition
            \Z:=egm_condition
            \MaxSpeedDeviation:=1000.0;

        EGMRunPose egm_id, EGM_STOP_HOLD, \X \Y \Z \CondTime:=1;

        EGMReset egm_id;
    ENDPROC
ENDMODULE
