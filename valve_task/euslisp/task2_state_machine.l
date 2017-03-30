#!/usr/bin/env roseus

(ros::roseus "task2_state_machine")
(load "package://valve_task/euslisp/start-roseus-aero-mbzirc.l")
(load "package://mbzirc_task2_control/euslisp/running-arena-side-motion.l")
(load "package://mbzirc_task2_control/euslisp/approach-panel.l")
(load "package://mbzirc_task2_control/euslisp/align-with-panel.l")
(load "package://mbzirc_task2_control/euslisp/laser-align-with-panel.l")
(load "package://mbzirc_task2_control/euslisp/catch-wrench.l")
(load "package://valve_task/euslisp/fit-wrench-to-shaft-aero.l")
(load "package://valve_task/euslisp/aero-arm-turn-wrench-torso.l")
(load "package://valve_task/euslisp/laser-panel-detector.l")

(setq targetDistFromWall 4)
(setq startPanelY 4.0) ;; how far the panel is in the y direction from the start point, in meters.

(defun debugPrint (msg)
    (print msg)
    )

 (setq currentState "goToInitPosition")
;; (setq currentState "midRange")
;; (setq currentState "alignBaseWithShaft")
;;(setq currentState "laserFarRange")

;;;;; the main state machine ! ;;;;;
(defun exec_task2 ()

    (setq isExit nil)
    (while (not (equal currentState "exit"))
        (cond
         ((equal currentState "laserFarRange") (laser-far-range-state))
         ((equal currentState "runArena") (run-arena-state))
         ((equal currentState "aligning") (aligning-state))
         ((equal currentState "pickWrench") (pick-wrench-state))
         ((equal currentState "alignBaseWithShaft") (align-base-with-shaft-state))
         ((equal currentState "alignWrenchWithShaft") (align-wrench-with-shaft-state))
         ((equal currentState "turnWrench") (turn-wrench-state))
         )
        )
    (format t "exiting task2 state machine. Task complete.")
    )



;;;;;;;;;;;;; Laser Far Range State ;;;;;;;;;;;;;
(defun laser-far-range-state ()

    (debugPrint "entered laser-far-range-state")
    ;;;; Execute the state ;;;;

    (setq farTargetDistFromWall 10.0)
    (move-to-wall-front farTargetDistFromWall 200)
    
    (setq laser-far-range-state-result (far-range-approach targetDistFromWall startPanelY))
    (cond
     (laser-far-range-state-result (setq currentState "exit"))
     ((not laser-far-range-state-result) (setq currentState "exit"))
    )
    (debugPrint "exiting laser-far-range-state")
    )

;;;;;;;;;;;;;  Aligning State ;;;;;;;;;;;;;
(defun aligning-state ()
    
    (debugPrint "entered aligning-state")
    ;;;; Execute the state ;;;;
    (setq aligning-state-result (align-with-panel :use-ri main-use-ri :auto main-auto))
    ;;(setq aligning-state-result (laser-align-with-panel (- 900 dim) 200))
    
    ;;;; Goto next state based on te execution result of this state
    (cond
     (aligning-state-result (progn (task2recogpose)
                                   (setq currentState "pickWrench")))
     ((not aligning-state-result) (progn (resetdrivingpose)
                                         (setq currentState "midRange"))) ;;todo
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
     ((not pick-wrench-state-result)  (setq currentState "aligning")))
    (debugPrint "exiting pick-wrench-state")
    )

;;;;;;;;;;;;;  Align Base With Shaft State ;;;;;;;;;;;;;
(defun align-base-with-shaft-state ()
    
    (debugPrint "entered align-base-with-shaft-state")
    ;;;; Execute the state ;;;;
    (setq align-base-with-shaft-state-result (align-with-panel :target-offset #f(-500 505 0) :use-ri main-use-ri :auto main-auto))
    ;;(setq align-base-with-state-result (laser-align-with-panel (- 800 dim) 505))
    
    ;;no feedback
    ;;(setq align-base-with-shaft-state-result t)
    ;;(move-base-side -100 :use-ri main-use-ri)
    ;;(move-base 305 0 :use-ri main-use-ri)
    
    ;;;; Goto next state based on te execution result of this state
    (cond
     (align-base-with-shaft-state-result (progn
                                             (resettask2pose)
                                             (setq currentState "alignWrenchWithShaft")))
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
     ((not align-wrench-with-shaft-state-result) (progn
                                                     (task2RecogPose)
                                                     (setq currentState "alignBaseWithShaft"))) ;;@TODO: handle failure case
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
     ((not turn-wrench-state-result) (setq currentState "alignBaseWithShaft"))
     )
    (debugPrint "exiting turn-wrench-state")
    )
