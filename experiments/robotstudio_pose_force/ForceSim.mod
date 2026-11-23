MODULE ForceSim
    ! NORMAL non-motion task T_FORCE. Synthetic sensor, NOT ABB FCAct.
    PERS bool virtualConfirmed:=FALSE;
    PERS bool simulationStop:=FALSE;
    PERS num forceHeartbeat:=0;
    PERS bool correctionEnabled:=FALSE;
    PERS num referenceN:=10;
    PERS num simulatedN:=10;
    PERS num correctionSign:=1;
    VAR socketdev channel;
    VAR num seq:=0;
    VAR num filteredN:=10;
    VAR num integral:=0;
    VAR num errorN;
    VAR num proposed;
    VAR num displacement:=0;
    VAR num change;
    VAR string packet;
    VAR string reply;
    VAR string sender;
    VAR num senderPort;
    PROC main()
        IF NOT virtualConfirmed THEN
            TPWrite "Synthetic force task. Virtual controller only.";
            RETURN;
        ENDIF
        simulationStop:=FALSE;
        seq:=0;
        displacement:=0;
        integral:=0;
        filteredN:=simulatedN;
        IF Abs(correctionSign)<>1 THEN
            TPWrite "correctionSign must be +1 or -1";
            RETURN;
        ENDIF
        SocketCreate channel\UDP;
        SocketBind channel,"127.0.0.1",6514;
        WHILE TRUE DO
            filteredN:=filteredN+0.2*(simulatedN-filteredN);
            errorN:=referenceN-filteredN;
            IF correctionEnabled THEN
                proposed:=0.02*errorN+integral+0.01*errorN*0.02;
                ! Conditional integration = anti-windup at +/-0.5 mm.
                IF Abs(proposed)<0.5 THEN
                    integral:=integral+0.01*errorN*0.02;
                ENDIF
                proposed:=correctionSign*(0.02*errorN+integral);
            ELSE
                integral:=0;
                proposed:=0;
            ENDIF
            proposed:=Max(-0.5,Min(0.5,proposed));
            change:=Max(-0.01,Min(0.01,proposed-displacement));
            displacement:=displacement+change;
            seq:=seq+1;
            packet:=NumToStr(seq,0)+","+NumToStr(displacement,5);
            SocketSendTo channel,"127.0.0.1",6513\Str:=packet;
            SocketReceiveFrom channel\Str:=reply,sender,senderPort\Time:=0.2;
            IF sender<>"127.0.0.1" OR senderPort<>6513 THEN
                simulationStop:=TRUE;
                SocketClose channel;
                RETURN;
            ENDIF
            IF reply="DONE,"+NumToStr(seq,0) THEN
                simulationStop:=TRUE;
                SocketClose channel;
                RETURN;
            ENDIF
            IF reply<>"RUN,"+NumToStr(seq,0) THEN
                simulationStop:=TRUE;
                SocketClose channel;
                RETURN;
            ENDIF
            forceHeartbeat:=forceHeartbeat+1;
            WaitTime 0.02;
        ENDWHILE
    ERROR
        simulationStop:=TRUE;
        SocketClose channel;
        RAISE;
    ENDPROC
ENDMODULE
