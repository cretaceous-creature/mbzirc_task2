#!/usr/bin/env roseus

(ros::roseus "task2_state_machine")
(load "package://mbzirc_task2_control/euslisp/running-arena.l")
(load "package://mbzirc_task2_control/euslisp/approach-panel.l")
(load "package://mbzirc_task2_control/euslisp/align-with-panel.l")
(load "package://mbzirc_task2_control/euslisp/catch-wrench.l")
(load "package://mbzirc_task2_control/euslisp/start-task.l")
(load "package://valve_task/euslisp/fit-wrench-to-shaft-aero.l")
(load "package://valve_task/euslisp/aero-arm-turn-wrench-torso.l")

;; initial guess of position of the panel
(setq init_x 40000)
(setq init_y 0)
(setq init_rot 120)
(setq rot-dir 0)
(setq start-look-angle -30)
(setq end-look-angle 30)
(setq wrench_target 4) ;; 3 for 19mm, 4 for 22mm
(setq main-use-ri t)
(setq main-auto t)

(start-task)

;;;;; the main state machine ! ;;;;;
(defun exec_task2 ()

    (setq isExit nil)
    (setq currentState "goToInitPosition")
    (while (not isExit)
        (cond
         ((equal currentState "goToInitPosition") (go-to-init-position-state init_x init_y))
         ((equal currentState "runArena") (run-arena-state))
         ((equal currentState "farRange") (far-range-state))
         ((equal currentState "farRangeAround") (far-range-around-state))
         ((equal currentState "midRange") (mid-range-state))
         ((equal currentState "shortRange") (short-range-state))
         ((equal currentState "aligning") (aligning-state))
         ((equal currentState "pickWrench") (pick-wrench-state))
         ((equal currentState "alignBaseWithShaft") (align-base-with-shaft-state))
         ((equal currentState "alignWrenchWithShaft") (align-wrench-with-shaft-state))
         ((equal currentState "turnWrench") (turn-wrench-state))
         )
        )
    )

;;;;;;;;;;;;;  Go to Init Position State ;;;;;;;;;;;;;
(defun go-to-init-position-state (init_x init_y)

    (debugPrint "entered go-to-init-position-state")
    ;;;;; Execute the state ;;;;
    ;; navigate to the inital guessed position of the panel
    (setq go-to-init-position-state-result (go-to-init-position :init_x init_x :init_y init_y :use-ri main-use-ri))

    ;;;; Goto next state based on the execution result of this state
    (cond
     (go-to-init-position-state-result (setq currentState "farRange"))
     ((not go-to-init-position-state-result) (setq currentState "exit"))
     )
    (debugPrint "exiting go-to-init-position-state")
    )

;;;;;;;;;;;;;  Go to Init Position State ;;;;;;;;;;;;;
(defun run-arena-state ()

    (debugPrint "entered run-arena-state")
    ;;;;; Execute the state ;;;;
    ;; navigate to the inital guessed position of the panel
    (setq run-arena-state-result (go-ro-next-position :use-ri main-use-ri))

    ;;;; Goto next state based on the execution result of this state
    (cond
     (run-arena-state-result (setq currentState "farRange"))
     ((not run-arena-state-result) (setq currentState "exit"))
     )
    (debugPrint "exiting run-arena-state")
    )

;;;;;;;;;;;;;  Far Range State ;;;;;;;;;;;;;
(defun far-range-state ()

    (debugPrint "entered far-range-state")
    ;;;; Execute the state ;;;;
    (setq far-range-state-result (long-approach :auto main-auto :use-ri main-use-ri :start-angle start-look-angle :end-angle end-look-angle))

    ;;;; Goto next state based on te execution result of this state
    (cond
     (far-range-state-result (setq currentState "midRange"))
     ((not far-range-state-result) (setq currentState "farRangeAround")) ;;todo
     )
    (debugPrint "exiting far-range-state")
    )

;;;;;;;;;;;;;  Far Range Around State ;;;;;;;;;;;;;
(defun far-range-around-state ()

    (debugPrint "entered far-range-around-state")
    ;;;; Execute the state ;;;;
    (setq far-range-around-state-result (long-approach :auto main-auto :use-ri main-use-ri :start-angle -90 :end-angle 90))

    ;;;; Goto next state based on te execution result of this state
    (cond
     (far-range-around-state-result (setq currentState "midRange"))
     ((not far-range-around-state-result) (setq currentState "runArena")) ;;todo
     )
    (debugPrint "exiting far-range-around-state")
    )

;;;;;;;;;;;;;  Mid Range State ;;;;;;;;;;;;;
(defun mid-range-state ()

    (debugPrint "entered mid-range-state")
    ;;;; Execute the state ;;;;
    (setq mid-range-state-result (middle-approach :use-ri main-use-ri :auto main-auto))

    ;;;; Goto next state based on te execution result of this state
    (cond
     (mid-range-state-result (setq currentState "shortRange"))
     ((not mid-range-state-result) (setq currentState "farRange"))
     )
    (debugPrint "exiting mid-range-state")
    )

;;;;;;;;;;;;;  Short Range State ;;;;;;;;;;;;;
(defun short-range-state ()

    (debugPrint "entered short-range-state")
    ;;;; Execute the state ;;;;
    (setq short-range-state-result (short-approach :use-ri main-use-ri :auto main-auto :rot-dir rot-dir))

    ;;;; Goto next state based on te execution result of this state
    (cond
     (short-range-state-result (setq currentState "aligning"))
     ((not short-range-state-result) (setq currentState "farRange"))
     )
    (debugPrint "exiting short-range-state")
    )

;;;;;;;;;;;;;  Aligning State ;;;;;;;;;;;;;
(defun aligning-state ()

    (debugPrint "entered aligning-state")
    ;;;; Execute the state ;;;;
    (setq aligning-state-result (align-with-panel :use-ri main-use-ri :auto main-auto))

    ;;;; Goto next state based on te execution result of this state
    (cond
     (aligning-state-result (setq currentState "pickwrench"))
     ((not aligning-state-result) (setq currentState "farRange"))
     )
    (debugPrint "exiting aligning-state")
    )

;;;;;;;;;;;;;  Pick Wrench State ;;;;;;;;;;;;;
(defun pick-wrench-state ()

    (debugPrint "entered pick-wrench-state")
    ;;;; Execute the state ;;;;
    (setq pick-wrench-state-result (full-pick wrench_target :auto main-auto :ready? t))

    ;;;; Goto next state based on te execution result of this state
    (cond
     (pick-wrench-state-result (setq currentState "alignBaseWithShaft"))
     ((not pick-wrench-state-result) (setq currentState "aligning"))
     )
    (debugPrint "exiting pick-wrench-state")
    )

;;;;;;;;;;;;;  Align Base With Shaft State ;;;;;;;;;;;;;
(defun align-base-with-shaft-state ()

    (debugPrint "entered pick-wrench-state")
    ;;;; Execute the state ;;;;
    (setq align-base-with-shaft-state-result (align-with-panel :target-offset #f(-550 505 0) :use-ri main-use-ri :auto main-auto))

    ;;;; Goto next state based on te execution result of this state
    (cond
     (align-base-with-shaft-state-result (setq currentState "alignWrenchWithShaft"))
     ((not align-base-with-shaft-state-result) (setq currentState "aligning"))
     )
    (debugPrint "exiting align-base-with-shaft-state")
    )

;;;;;;;;;;;;;  Align Wrench With Shaft State ;;;;;;;;;;;;;
(defun align-wrench-with-shaft-state ()

    (debugPrint "entered align-wrench-with-shaft-state")
    ;;;; Execute the state ;;;;
    (setq align-wrench-with-shaft-state-result (align-wrench-with-shaft))

    ;;;; Goto next state based on te execution result of this state
    (cond
     (align-wrench-with-shaft-state-result (setq currentState "turnWrench"))
     ((not align-wrench-with-shaft-state-result) (setq currentState "exit")) ;;@TODO: handle failure case 
     )
    (debugPrint "exiting align-wrench-with-shaft-state")
    )

;;;;;;;;;;;;;  Turn Wrench State ;;;;;;;;;;;;;
(defun turn-wrench-state ()

    (debugPrint "entered turn-wrench-state")
    ;;;; Execute the state ;;;;
    (setq turn-wrench-state-result (turn-shaft-full-turn)) 

    ;;;; Goto next state based on te execution result of this state
    (cond
     (turn-wrench-state-result (setq currentState "exit"))
     ((not turn-wrench-state-result) (setq currentState "aligning"))
     )
    (debugPrint "exiting turn-wrench-state")
    )

(defun debugPrint (msg)
    (print msg)
    )
