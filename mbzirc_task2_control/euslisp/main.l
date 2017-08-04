#!/usr/bin/env roseus

(ros::roseus "task2_state_machine")
(load "package://valve_task/euslisp/start-roseus-aero-mbzirc.l")
(load "package://mbzirc_task2_control/euslisp/running-arena-side-motion.l")
(load "package://mbzirc_task2_control/euslisp/approach-panel.l")
(load "package://mbzirc_task2_control/euslisp/align-with-panel.l")
(load "package://mbzirc_task2_control/euslisp/laser-align-with-panel.l")
(load "package://mbzirc_task2_control/euslisp/catch-wrench.l")
(load "package://mbzirc_task2_control/euslisp/start-task.l")
(load "package://valve_task/euslisp/fit-wrench-to-shaft-aero.l")
(load "package://valve_task/euslisp/aero-arm-turn-wrench-torso.l")
(load "package://valve_task/euslisp/laser-panel-detector.l")

(setq targetDistFromWall 4.0)
(setq startPanelY -2.0) ;; how far the panel is in the y direction from the start point, in meters.

;; search front of panel param
(setq rot-dir -1) ;;1 to left, -1 to right turn

(setq wrench_target 3) ;; 3 for 19mm, 4 for 22mm
(setq main-use-ri t)
(setq main-auto t)

;; side motion param
(setq max-side-motion-time 3)
(setq side-motion-onetime-dist 2500)
(setq x-motion-onetime-dist 1500)
(setq first-side-motion-dir -1);; 1 to right, -1 to left

(setq side-motion-count 0)
(setq side-motion-dir first-side-motion-dir)


;; initial guess of position of the panel
(setq init_x 21000)
(setq init_y1 0)
(setq init_y2 9000)


(start-task)

(defun debugPrint (msg)
    (print msg)
    )

;; (setq currentState "goToInitPosition")
;; (setq currentState "pickWrench")
(setq currentState "farRange")
;; (setq currentState "midRange")
;; (setq currentState "alignBaseWithShaft")
;;(setq currentState "laserFarRange")

;;;;; the main state machine ! ;;;;;
(defun exec_task2 ()

    (setq isExit nil)
    (while (not (equal currentState "exit"))
        (cond
         ((equal currentState "goToInitPosition") (go-to-init-position-state))
         ((equal currentState "laserFarRange") (laser-far-range-state))
         ((equal currentState "runArena") (run-arena-state))
         ((equal currentState "farRange") (far-range-state))
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
(defun go-to-init-position-state ()

    (debugPrint "entered go-to-init-position-state")
    ;;;;; Execute the state ;;;;
    ;; navigate to the inital guessed position of the panel
    (setq go-to-init-position-state-result (go-to-init-position :x init_x :y1 init_y1 :y2 init_y2 :use-ri main-use-ri))

    ;;;; Goto next state based on the execution result of this state
    (cond
     (go-to-init-position-state-result (progn (unix::sleep 15)
                                              (task2recogpose)
                                              (setq currentState "midRange")))
     ((not go-to-init-position-state-result) (setq currentState "exit"))
     )
    (debugPrint "exiting go-to-init-position-state")
    )

;;;;;;;;;;;;;  Go to Init Position State ;;;;;;;;;;;;;
(defun run-arena-state ()

    (debugPrint "entered run-arena-state")
    ;;;;; Execute the state ;;;;
    ;; navigate to the inital guessed position of the panel
    (setq run-arena-state-result (side-motion :use-ri main-use-ri))

    ;;;; Goto next state based on the execution result of this state
    (cond
     (run-arena-state-result (progn (task2recogpose)(setq currentState "midRange")))
     ((not run-arena-state-result) (setq currentState "exit"))
     )
    (debugPrint "exiting run-arena-state")
    )


;;;;;;;;;;;;; Laser Far Range State ;;;;;;;;;;;;;
(defun laser-far-range-state ()

    (debugPrint "entered laser-far-range-state")
    ;;;; Execute the state ;;;;

    (setq farTargetDistFromWall 10.0)
    (move-to-wall-front farTargetDistFromWall 200)
    
    (setq laser-far-range-state-result (far-range-approach targetDistFromWall startPanelY))
    (cond
     (laser-far-range-state-result (progn (unix::sleep 10)
                                          (setq currentState "midRange")))
     ((not laser-far-range-state-result) (setq currentState "exit"))
    )
    (debugPrint "exiting laser-far-range-state")
    )

;;;;;;;;;;;;;  Far Range State ;;;;;;;;;;;;;
(defun far-range-state ()
    
    (debugPrint "entered far-range-state")
    ;;;; Execute the state ;;;;
    (setq far-range-state-result (long-approach :use-ri main-use-ri :auto main-auto ))
    
    ;;;; Goto next state based on te execution result of this state
    (cond
     (far-range-state-result (setq currentState "midRange"))
     ((not far-range-state-result) (setq currentState "runArena")))
     )
    (debugPrint "exiting far-range-state")
    )

;;;;;;;;;;;;;  Middle Range State ;;;;;;;;;;;;;
(defun mid-range-state ()
    
    (debugPrint "entered mid-range-state")
    ;;;; Execute the state ;;;;
    ;; (setq mid-range-state-result (middle-approach-look-right :use-ri main-use-ri :auto main-auto ))
        (setq mid-range-state-result (middle-approach :use-ri main-use-ri :auto main-auto ))
    
    ;;;; Goto next state based on te execution result of this state
    (cond
     (mid-range-state-result (setq currentState "shortRange"))
     ((not mid-range-state-result) (setq currentState "runArena")))
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
     ((not short-range-state-result) (progn (task2recogpose)
                                            (setq currentState "exit")))
     )
    (debugPrint "exiting short-range-state")
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
     ((not aligning-state-result) (setq currentState "aligning")) ;;todo
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
     ((not pick-wrench-state-result)  (progn (task2RecogPose)
                                             (setq currentState "aligning"))))
    (debugPrint "exiting pick-wrench-state")
    )

;;;;;;;;;;;;;  Align Base With Shaft State ;;;;;;;;;;;;;
(defun align-base-with-shaft-state ()

    (debugPrint "entered align-base-with-shaft-state")
    ;;;; Execute the state ;;;;
    (setq align-base-with-shaft-state-result (align-with-panel :target-offset #f(-530 460 0)
                                                               :target-rot 0.0 :use-ri main-use-ri :auto main-auto))

    ;;;; Goto next state based on te execution result of this state
    (cond
     (align-base-with-shaft-state-result (progn (unix::sleep 2)
                                             (setq currentState "alignWrenchWithShaft")))
     ((not align-base-with-shaft-state-result) (setq currentState "alignBaseWithShaft"))
     )
    (debugPrint "exiting align-base-with-shaft-state")
    )

;;;;;;;;;;;;;  Align Wrench With Shaft State ;;;;;;;;;;;;;
(defun align-wrench-with-shaft-state ()
    
    (debugPrint "entered align-wrench-with-shaft-state")
    ;;;; Execute the state ;;;;
    (unix::sleep 3)
    ;;(setq align-wrench-with-shaft-state-result (align-wrench-with-shaft))
    (setq align-wrench-with-shaft-state-result (align-wrench-with-shaft-from-recog-pose))
    
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
     (turn-wrench-state-result (progn
                                   (format t "~%You did it!! Congratulations!!~%~%")
                                   (task2RecogPose)
                                   (setq currentState "exit")))
     ((not turn-wrench-state-result) (progn
                                         (task2RecogPose)
                                         (setq currentState "alignBaseWithShaft")))
     )
    (debugPrint "exiting turn-wrench-state")
    )
