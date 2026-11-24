MODULE MillingPose6510
    ! Load only into a NEW virtual IRB1600-10/1.45 controller, T_ROB1.
    ! This module does not replace RobotStudio/ModuleEGM.mod.
    PERS bool virtualConfirmed:=FALSE;
    PERS tooldata simTool:=[TRUE,[[0,0,280],[1,0,0,0]],[0.05,[0,0,100],[1,0,0,0],0,0,0]];
    CONST robtarget simHome:=[[750,0,600],[0,1,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS num runSeconds:=5;
    VAR egmident id;
    VAR bool acquired:=FALSE;
    VAR pose baseFrame:=[[0,0,0],[1,0,0,0]];
    VAR egm_minmax linear:=[-0.1,0.1];
    VAR egm_minmax angular:=[-0.5,0.5];
    PROC main()
        IF NOT virtualConfirmed THEN
            TPWrite "Virtual controller only. Read experiment README.";
            RETURN;
        ENDIF
        acquired:=FALSE;
        IF runSeconds<3 OR runSeconds>243 THEN
            TPWrite "Set runSeconds from the PC startup message.";
            RETURN;
        ENDIF
        MoveJ simHome,v20,fine,simTool;
        EGMGetId id;
        acquired:=TRUE;
        ! Use station UdpUc device: UCdevice, 127.0.0.1:6510.
        EGMSetupUC ROB_1,id,"default","UCdevice"\Pose;
        EGMActPose id\Tool:=simTool\WObj:=wobj0,
            baseFrame,EGM_FRAME_BASE,baseFrame,EGM_FRAME_BASE
            \X:=linear\Y:=linear\Z:=linear
            \Rx:=angular\Ry:=angular\Rz:=angular
            \LpFilter:=20\SampleRate:=4\MaxSpeedDeviation:=50;
        EGMRunPose id,EGM_STOP_HOLD\NoWaitCond\X\Y\Z\Rx\Ry\Rz
            \CondTime:=300\RampInTime:=0.1;
        WaitTime runSeconds;
        EGMStop id,EGM_STOP_HOLD;
        EGMWaitCond id;
        EGMReset id;
        acquired:=FALSE;
    ERROR
        IF acquired THEN
            EGMReset id;
        ENDIF
        RAISE;
    ENDPROC
ENDMODULE
