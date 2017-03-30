#!/usr/bin/env roseus

(load "package://aero_utils/euslisp/aero-arm-single-wrist-utils.l")
(load "package://valve_task/euslisp/fit-wrench-to-shaft-aero.l")
(load "package://valve_task/euslisp/aero-arm-turn-wrench-arm.l")
(load "package://valve_task/euslisp/aero-arm-turn-wrench-torso.l")

(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "std_srvs")

(setq *robot* *aero*)


(setq debug t)

(defun prompt (msg)
    (if debug
            (progn
                (print msg)
                (read-char)))
    )


(defun callbackJoy (joyState)
    ;; (print "in callbackJoy")

    ;; make sure only use newer msg, and not old msg with using UDP communication
    (format t "prevMsgTime ~a~%" prevMsgTime)
    (format t "(send joyState :header :stamp) ~a~%"  (send joyState :header :stamp))
    (if (not (ros::time< prevMsgTime (send joyState :header :stamp)))
            (setq joyState (instance sensor_msgs::Joy :init))
        )

    ;; euslisp is calling this mulitple time in different threads before the previous callbacks returns.
    ;; Need to prevent problem with this bug!!
    (if executing_joycb
            (return-from callbackJoy)
        )
    (if (equal (length (send joyState :axes)) 0) ;; when joy node dies 
            (return-from callbackJoy)
        )

    (setq executing_joycb t)
    (setq isMovingBase nil)


    (setq LStick_LR (aref (send joyState :axes) 0))
    (setq LStick_UD (aref (send joyState :axes) 1))
    (setq RStick_LR (aref (send joyState :axes) 2))
    (setq RStick_UD (aref (send joyState :axes) 3))

    (setq Accl_LR (aref (send joyState :axes) 4))
    (setq Accl_FB (aref (send joyState :axes) 5))
    (setq Accl_UD (aref (send joyState :axes) 6))
    (setq Yaw (aref (send joyState :axes) 7))

    (setq DPad_U (aref (send joyState :axes) 8))
    (setq DPad_R (aref (send joyState :axes) 9))
    (setq DPad_D (aref (send joyState :axes) 10))
    (setq DPad_L (aref (send joyState :axes) 11))
    
    (setq L2 (aref (send joyState :axes) 12))
    (setq R2 (aref (send joyState :axes) 13))
    (setq L1 (aref (send joyState :axes) 14))
    (setq R1 (aref (send joyState :axes) 15))

    (setq Triangle (aref (send joyState :axes) 16))
    (setq Circle (aref (send joyState :axes) 17))
    (setq Cross (aref (send joyState :axes) 18))
    (setq Square (aref (send joyState :axes) 19))
    
    (setq Select (aref (send joyState :buttons) 0))
    (setq L3 (aref (send joyState :buttons) 1))
    (setq R3 (aref (send joyState :buttons) 2))
    (setq Start (aref (send joyState :buttons) 3))

    (setq PSButton (aref (send joyState :buttons) 16))

    ;;;;;;; hand orientation ;;;;;;;;

    ;; (setq normal_vector #f(1 0 0)) ;; may want to change this to the normal vector of the panel later
    ;; (setq zaxis #f(0 0 1))

    ;; Compute hand orientation. Only allow 1 Dof Rotation about xaxis. 
    ;; (setq theta (atan2 (- Accl_UD) (+ Accl_LR)))
    ;; (setq panel_normal_orientation (make-matrix 3 3 (list normal_vector (v* zaxis normal_vector) zaxis)))
    ;; (setq rot (m* (rotation-matrix theta #f(-1 0 0)) panel_normal_orientation))

    ;; (send *tfb* :send-transform (make-coords :pos #f(0 -1000 0) :rot panel_normal_orientation) "CHEST_LINK0" "panel_orientation")
    ;; (send *tfb* :send-transform (make-coords :pos #f(500 -1500 0) :rot rot) "CHEST_LINK0" "ps3joy_aligned")

    ;; (send *robot* :rarm :inverse-kinematics (make-coords :pos rarm-endpos-worldpos :rot rot)
          ;; :thre 0.1
         ;; )


    ;;;;;;; end hand orientation ;;;;;;;


    ;;;;;;;;;;;;;; buttons ;;;;;;;;;;;;;;;;

    (setq trigger_thre -0.2)
    (setq button_thre -0.2)
    (setq stick_thre 0.01)


    ;; E-stop function
    (if (and (< L1 trigger_thre)
             (< L2 trigger_thre)
             (equal Select 1))
            (progn
                (arm-servo-disable)
                (wheels-servo-disable)
                (gripper-servo-disable)
                (print "E-stop disable")
                )
        )
    ;; recover from E-stop
    (if (and (< R1 trigger_thre)
             (< R2 trigger_thre)
             (equal Start 1))
            (progn
                (arm-servo-enable)
                (wheels-servo-enable)
                (gripper-servo-enable)
                (print "E-stop enable")
                )
        )

    
    (setq mode nil)
    (cond
     ;; ((and (< L1 trigger_thre) (> L2 trigger_thre) (> R1 trigger_thre) (> R2 trigger_thre))
     ((and (< L1 trigger_thre) (> R1 trigger_thre) (> R2 trigger_thre)) ;; L2 can be off, or on for fine mode
      (setq mode "base_mode"))
     ((and (< L2 trigger_thre) (> L1 trigger_thre) (> R1 trigger_thre) (> R2 trigger_thre))
      (setq mode "torso_mode"))
     ((and (< R2 trigger_thre) (> L1 trigger_thre) (> L2 trigger_thre) (> R1 trigger_thre))
      (setq mode "rarm_mode"))
     ((and (< L2 trigger_thre) (< R2 trigger_thre) (> L1 trigger_thre) (> R1 trigger_thre))
      (setq mode "dual_arm_mode"))
     ((and (< R1 trigger_thre) (> L1 trigger_thre) (> L2 trigger_thre) (> R2 trigger_thre))
      (setq mode "head_mode"))
     )
       

    (cond
     ((equal mode "rarm_mode")
      (progn
          (if (and (< prev_Circle button_thre) (> Circle button_thre))
                  (progn
                      (gripper-servo-disable)
                      (print "gripper-servo-disable")
                      )
              )
          (if (and (< prev_Square button_thre) (> Square button_thre))
                  (progn
                      (gripper-servo-enable)
                      (gripper-servo-pos 0 3)
                      (print "gripper-servo-enable")
                      )
              )
          (if (< Triangle button_thre)
                  (progn
                      (gripper-servo-enable)
                      (setq gripper-command-pos (+ gripper-current-pos 10))
                      (format t "gripper current pos ~a~%" gripper-current-pos)
                      (format t "gripper command pos ~a~%" gripper-command-pos)
                      (gripper-servo-pos gripper-command-pos 4000)
                      (setq gripper-current-pos gripper-command-pos)
                      (unix:usleep 200000)
                      )
              )
          (if (< Cross button_thre)
                  (progn
                      (gripper-servo-enable)
                      (setq gripper-command-pos (- gripper-current-pos 10))
                      (format t "gripper current pos ~a~%" gripper-current-pos)
                      (format t "gripper command pos ~a~%" gripper-command-pos)
                      (gripper-servo-pos gripper-command-pos 4000)
                      (setq gripper-current-pos gripper-command-pos)
                      (unix:usleep 200000)
                      )
              )
          (if (or (> (abs RStick_UD) stick_thre) (> (abs LStick_LR) stick_thre) (> (abs LStick_UD) stick_thre))
                  (progn
                      (setq moveArmGainX 25)
                      (setq moveArmGainY 15)
                      (setq moveArmGainZ 25)


                      (if (> (abs RStick_UD) stick_thre)
                              (progn
                                  (if (> RStick_UD 0)
                                          (setq RStick_UD_adjusted (- RStick_UD stick_thre))
                                  (setq RStick_UD_adjusted (+ RStick_UD stick_thre)))
                                  )
                          (setq RStick_UD_adjusted 0)
                          )
                      (if (> (abs LStick_LR) stick_thre)
                              (progn
                                  (if (> LStick_LR 0)
                                          (setq LStick_LR_adjusted (- LStick_LR stick_thre))
                                  (setq LStick_LR_adjusted (+ LStick_LR stick_thre)))
                                  )
                          (setq LStick_LR_adjusted 0)
                          )
                      (if (> (abs LStick_UD) stick_thre)
                              (progn
                                  (if (> LStick_UD 0)
                                          (setq LStick_UD_adjusted (- LStick_UD stick_thre))
                                  (setq LStick_UD_adjusted (+ LStick_UD stick_thre)))
                                  )
                          (setq LStick_UD_adjusted 0)
                          )  
                      
                      (move-arm (float-vector (* RStick_UD_adjusted moveArmGainX)
                                              (* LStick_LR_adjusted moveArmGainY)
                                              (* LStick_UD_adjusted moveArmGainZ)))
                      )
              )
          (if (< DPad_U button_thre)
                  (progn
                      ;;(move-arm-to-start-pose)
                      ;; (lift-arm-up)
                      (readyturnwrenchtorso)
                      (matchsimwithrobot)
                      (setq in_progress_target_rarm_pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos))
                      (setq in_progress_target_rarm_rot (send (send *robot* :rarm :end-coords :copy-worldcoords) :rot))
                      (setq in_progress_target_shoulder_angle (send *robot* :rarm :shoulder-y :joint-angle))
                      )
              )
          (if (< DPad_L button_thre)
                  (progn
                      ;; (align-wrench-with-shaft)
                      ;; (turnwrenchtorso)
                      ;; (matchsimwithrobot)
                      ;; (setq in_progress_target_rarm_pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos))
                      ;; (setq in_progress_target_rarm_rot (send (send *robot* :rarm :end-coords :copy-worldcoords) :rot))
                      ;; (setq in_progress_target_shoulder_angle (send *robot* :rarm :shoulder-y :joint-angle))
                      (print "button combo not mapped")
                      )
              )
          (if (< DPad_R button_thre)
                  (progn
                      ;; (align-wrench-with-shaft)
                      (turn-shaft-disengage-move-arm)
                      (unix:sleep 4)
                      (matchsimwithrobot)
                      (setq in_progress_target_rarm_pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos))
                      (setq in_progress_target_rarm_rot (send (send *robot* :rarm :end-coords :copy-worldcoords) :rot))
                      (setq in_progress_target_shoulder_angle (send *robot* :rarm :shoulder-y :joint-angle))
                      )
              )
          (if (< DPad_D button_thre)
                  (progn
                      (if (boundp `arm-delta)
                              (setq arm-delta (fit-wrench-to-shaft arm-delta))
                          (setq arm-delta (fit-wrench-to-shaft))
                          )
                      ;; (format t "no function binded")
                      )
              )
          (if (and (equal prev_Select 1) (equal Select 0))
                  (progn
                      (send *robot* :rarm-shoulder-y :joint-angle -90)
                      (send *robot* :rarm-shoulder-p :joint-angle 0)
                      (send *robot* :rarm-elbow-p :joint-angle 0)
                      (send *robot* :rarm-wrist-p :joint-angle 0)
                      (send *robot* :rarm-wrist-y :joint-angle 0)
                      (send *robot* :rarm-dgripper :joint-angle 90)
                      (moverobot)
                      )
              )     
          )
      )
     ((equal mode "dual_arm_mode")
      (progn
          ;; (print "dual_arm_mode")
          (if (< DPad_L button_thre)
                  (progn
                      (print "reset task2 pose")
                      (resettask2pose)
                      (setq in_progress_target_rarm_pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos))
                      (setq in_progress_target_shoulder_angle (send *robot* :rarm :shoulder-y :joint-angle))
                      )
              )
          (if (< DPad_U button_thre)
                  (progn
                      (print "reset task2 pose arm only")
                      (resettask2poseArmOnly)
                      (setq in_progress_target_rarm_pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos))
                      (setq in_progress_target_shoulder_angle (send *robot* :rarm :shoulder-y :joint-angle))
                      )
              )
          (if (< DPad_R button_thre)
                  (progn
                      (print "task 2 recog pose")
                      (Task2RecogPose)
                      (setq in_progress_target_rarm_pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos))
                      (setq in_progress_target_shoulder_angle (send *robot* :rarm :shoulder-y :joint-angle))
                      )
              )
          (if (< DPad_D button_thre)
                  (progn
                      (turn-shaft-full-turn)
                      )
              )
          (if (and (< prev_Circle button_thre) (> Circle button_thre))
                  (progn
                      (open-hand :larm)
                      (open-hand :rarm)
                      )
              )
          (if (and (< prev_Square button_thre) (> Square button_thre))
                  (progn
                      (close-hand :larm)
                      (close-hand :rarm)
                      )
              )
          (if (equal Select 1)
                  (setq quit_teleop t)
              )
          )
      )
     ((equal mode "head_mode")
      (progn
          (if (or (> (abs RStick_UD) stick_thre) (> (abs LStick_LR) stick_thre))
                  (progn
                      (setq moveHeadGain 10)
                      (move-shoulder (* moveHeadGain LStick_LR)))
              )
          )
      )
     ((equal mode "base_mode")
      (progn
          (if (or (> (abs LStick_UD) stick_thre) (> (abs LStick_LR) stick_thre) (> (abs RStick_LR) stick_thre))
                  (progn
                      (if (< L2 trigger_thre) ;; if L2 pressed, fine mode 
                              (progn
                                  (setq moveBaseGainX 200)
                                  (setq moveBaseGainY 20)
                                  (setq moveBaseGainTheta 200)
                                  )
                          (progn ;; regular speed
                              (setq moveBaseGainX 10)
                              (setq moveBaseGainY 5)
                              (setq moveBaseGainTheta 10)
                              )
                          )    
                      (if (> (abs LStick_UD) stick_thre)
                              (progn
                                  (if (> LStick_UD 0)
                                          (setq LStick_UD_adjusted (- LStick_UD stick_thre))
                                  (setq LStick_UD_adjusted (+ LStick_UD stick_thre)))
                                  )
                          (setq LStick_UD_adjusted 0)
                          )
                      (if (> (abs LStick_LR) stick_thre)
                              (progn
                                  (if (> LStick_LR 0)
                                          (setq LStick_LR_adjusted (- LStick_LR stick_thre))
                                  (setq LStick_LR_adjusted (+ LStick_LR stick_thre)))
                                  )
                          (setq LStick_LR_adjusted 0)
                          )
                      (if (> (abs RStick_LR) stick_thre)
                              (progn
                                  (if (> RStick_LR 0)
                                          (setq RStick_LR_adjusted (- RStick_LR stick_thre))
                                  (setq RStick_LR_adjusted (+ RStick_LR stick_thre)))
                                  )
                          (setq RStick_LR_adjusted 0)
                          )
                      
                      (move-base (* LStick_UD_adjusted moveBaseGainX) (* LStick_LR_adjusted moveBaseGainY) (* RStick_LR_adjusted moveBaseGainTheta) 50)
                      (setq isMovingBase t)
                      (setq isMovedBaseZeroOnce nil)
                      )                
              )
          (if (and (< prev_Circle button_thre) (> Circle button_thre))
                  (progn
                      (wheels-servo-disable)
                      (print "wheels-servo-disable")
                      )
              )
          (if (and (< prev_Square button_thre) (> Square button_thre))
                  (progn
                      (wheels-servo-enable)
                      (print "wheels-servo-enable")
                      )
              )

          )
      )
     ((equal mode "torso_mode")
      (progn
          (if (or (> (abs LStick_UD) stick_thre) (> (abs RStick_LR) stick_thre))
                  (progn
                      (setq moveTorsoGain 10)
                      (if (> (abs LStick_UD) stick_thre)
                              (progn
                                  (if (< LStick_UD 0)
                                          (setq z_delta (- moveTorsoGain))
                                      (setq z_delta moveTorsoGain)
                                      )
                                  )
                          (setq z_delta 0)
                          )
                      (if (> (abs RStick_LR) stick_thre)
                              (progn
                                  (if (> RStick_LR 0)
                                          (setq x_delta moveTorsoGain)
                                      (setq x_delta (- moveTorsoGain))
                                      )
                                  )
                          (setq x_delta 0)
                          )
                      (move-torso (float-vector x_delta 0 z_delta))
                      )                
              )
          (if (< DPad_D button_thre)
                  (progn
                      (print "reset driving pose")
                      (resetDrivingPose)
                      (unix:sleep 4)
                      (setq in_progress_target_rarm_pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos))
                      (setq in_progress_target_rarm_rot (send (send *robot* :rarm :end-coords :copy-worldcoords) :rot))
                      (setq in_progress_target_shoulder_angle (send *robot* :rarm :shoulder-y :joint-angle))
                      )
              )
          (if (and (= prev_Select 0) (= Select 1))
                  (progn
                      (print "reset torso")
                      (moveWaist #f(0 0 0) :world 5000)
                      )
              )
          )
      )
     )
    ;; (if (and (equal prev_Select 1) (equal Select 0)) ;; 0 is also true in euslisp!!!!!!
    ;; (progn
    ;; (setq isMoveRobot (not isMoveRobot))
    ;; (if isMoveRobot
    ;; (print "isMoveRobot now true")
    ;; (print "isMoveRobot now false"))
    ;; ))
    
    
    

    
    ;;;;;;;;;;;;;; end buttons ;;;;;;;;;;;

    
    (if (boundp '*irtviewer*)
		(send *irtviewer* :draw-objects))
    

    (setq prev_LStick_LR LStick_LR)
    (setq prev_LStick_UD LStick_UD)
    (setq prev_RStick_UD RStick_UD)
    (setq prev_RStick_UD RStick_LR)
    
    (setq prev_Accl_LR Accl_LR)
    (setq prev_Accl_FB Accl_FB)
    (setq prev_Accl_UD Accl_UD)
    (setq prev_Yaw Yaw)
    
    (setq prev_DPad_U DPad_U)
    (setq prev_DPad_R DPad_R)
    (setq prev_DPad_D DPad_D)
    (setq prev_DPad_L DPad_L)
        
    (setq prev_L2 L2)
    (setq prev_R2 R2)
    (setq prev_L1 L1)
    (setq prev_R1 R1)

    (setq prev_Triangle Triangle)
    (setq prev_Circle Circle)
    (setq prev_Cross Cross)
    (setq prev_Square Square)

    ;; (print "setting prev")
    ;; (format t "prev_Square ~a~%" prev_Square)
    ;; (format t "Square ~a~%" Square)
    
    (setq prev_Select Select)
    (setq prev_L3 L3)
    (setq prev_R3 R3)
    (setq prev_Start Start)

    (setq prev_PSButton PSButton)

    (unless isMovingBase
        (unless isMovedBaseZeroOnce
            (progn
                (move-base 0 0 0 0)
                (setq isMovedBaseZeroOnce t)
                )
            )
        )
    (setq executing_joycb nil)
    (setq prev_seq curr_seq)
    (setq prevMsgTime (send joyState :header :stamp))
    )

(defun move-base (vel_x vel_y vel_theta time_msec)
    (setq twist_msg (instance geometry_msgs::TwistStamped :init))
    (send twist_msg :twist :linear :x vel_x)
    (send twist_msg :twist :linear :y vel_y)
    (send twist_msg :twist :angular :z vel_theta)
    (send twist_msg :header :stamp (instance ros::time :init :nsec (* time_msec 1000 1000)))
    
    (format t "sending x y theta time ~a ~a ~a ~a~%"
            (send twist_msg :twist :linear :x)
            (send twist_msg :twist :linear :y)
            (send twist_msg :twist :angular :z)
            (send twist_msg :header :stamp))
    
    (ros::publish "move_base_simple/velocity" twist_msg)
    )

(defun Move-arm (delta)
           
    (format t "delta ~a~%" delta)
    (format t "arm end coords worldpos ~a~%" (send *robot* :rarm :end-coords :worldpos))
    (setq in_progress_target_arm_pos in_progress_target_rarm_pos)
    (format t "in_progress_arm_target ~a~%" in_progress_target_arm_pos)

    
    ;; check to see that the new target is in the same direction as delta relative to the in progress target to prevent backlash
    (setq move-arm-thre 0.02)

    (setq in_progress_target_delta (send (send *robot* :rarm :end-coords) :inverse-transform-vector in_progress_target_arm_pos)) ;; convert to local coords
    (setq target-pos-diff (v- in_progress_target_delta delta))
    (format t "in_progress_target_delta ~a~%" in_progress_target_delta)
    (setq i 0) (send *robot* :rarm :end-coords) 
    (while (< i 3)
        (if (< (abs (elt in_progress_target_delta i)) move-arm-thre)
               (setf (aref in_progress_target_delta i) 0))
        (if (or (< 0 (* (elt target-pos-diff i) (elt delta i))) ;; avoid sending target that is closer to what has already been sent.
                (< (abs (elt delta i)) move-arm-thre)) ;; don't change the component of the target if the input is zero or too small to prevent drift.
                (setf (aref delta i) (aref in_progress_target_delta i)))
        (setq i (+ i 1))
        )
    (format t "delta after correction ~a~%" delta)
    (setq delta_world (send (send *robot* :rarm :end-coords) :rotate-vector delta))
    (format t "delta world ~a~%" delta_world) 
    (setq target-arm-coords (send *robot* :rarm :end-coords :copy-worldcoords))
    (send target-arm-coords :translate delta_world :world)
    (setq target-arm-pos (send (send target-arm-coords :copy-worldcoords) :pos)) ;; need to use copy-worldcoords as :worldpos returns a reference

    (format t "target arm world pos ~a~%" target-arm-pos)
    ;; (prompt "hit enter to continue")
    (if  (send *robot* :rarm :inverse-kinematics
               (make-coords :pos target-arm-pos
                            :rot (send (send *aero* :rarm :end-coords :copy-worldcoords) :rot))
               :thre 0.1
               :rthre 0.1)
            (progn 
                (moverobot 100)
                (unix::usleep 100000)
                (setq in_progress_target_rarm_pos target-arm-pos)
                )
        (setq in_progress_target_rarm_pos (send *robot* :rarm :end-coords :worldpos))
        )

    (setq in_progress_target_shoulder_angle (send *aero* :rarm :shoulder-y :joint-angle))
    (format t "~%~%")
    )


(defun move-shoulder (delta)
    
    (format t "delta ~a~%" delta)
    (format t "in_progress_shoulder_target ~a~%" in_progress_target_shoulder_angle)
    
    (setq tmp-angle-vec (send *robot* :angle-vector)) ;; need to save the robot angle vector before calling matchsimwithrobot first to prevet drifting
    (matchsimwithrobot) 
    (setq current-shoulder-angle (send *robot* :rarm :shoulder-y :joint-angle))

    (setq target-shoulder-angle (+ current-shoulder-angle delta))
    (format t "target-shoulder-angle ~a~%" target-shoulder-angle)
    (setq target-shoulder-angle-diff (- in_progress_target_shoulder_angle target-shoulder-angle))
    (format t "target-shoulder-angle-diff ~a~%" target-shoulder-angle-diff)
    (send *robot* :angle-vector tmp-angle-vec) ;; done calculations, restore model in sim
    
    ;; check to see that the new target is in the same direction as delta relative to the in progress target to prevent backlash
    (setq move-shoulder-thre 0.02)
    (if (or (< 0 (* target-shoulder-angle-diff delta)) ;; avoid sending target that is closer to what has already been sent.
            (< (abs delta) move-shoulder-thre)) ;; don't change the component of the target if the input is zero or too small to prevent drift.
            (setq target-shoulder-angle in_progress_target_shoulder_angle))
    (format t "target-shoulder-angle ~a~%" target-shoulder-angle)

    (send *robot* :rarm :shoulder-y :joint-angle target-shoulder-angle)
    (moverobot 100)
    
    (setq in_progress_target_shoulder_angle target-shoulder-angle)
    (setq in_progress_target_rarm_pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos))
    (setq in_progress_target_rarm_rot (send (send *robot* :rarm :end-coords :copy-worldcoords) :rot))
    )

(defun move-torso (delta)

    (format t "move-torso~%")
    (format t "delta ~a~%" delta)
    ;; (format t "in_progress_target ~a~%" in_progress_target_torso)
    
    ;; (setq tmp-angle-vec (send *robot* :angle-vector)) ;; need to save the robot angle vector before calling matchsimwithrobot first to prevet drifting
    ;; (matchsimwithrobot) 
    ;; (setq current-shoulder-angle (send *robot* :rarm :shoulder-y :joint-angle))

    ;; (setq target-shoulder-angle (+ current-shoulder-angle delta))
    ;; (format t "target-shoulder-angle ~a~%" target-shoulder-angle)
    ;; (setq target-shoulder-angle-diff (- in_progress_target_shoulder_angle target-shoulder-angle))
    ;; (format t "target-shoulder-angle-diff ~a~%" target-shoulder-angle-diff)
    ;; (send *robot* :angle-vector tmp-angle-vec) ;; done calculations, restore model in sim
    
    ;; check to see that the new target is in the same direction as delta relative to the in progress target to prevent backlash
    ;; (setq move-shoulder-thre 0.02)
    ;; (if (or (< 0 (* target-shoulder-angle-diff delta)) ;; avoid sending target that is closer to what has already been sent.
            ;; (< (abs delta) move-shoulder-thre)) ;; don't change the component of the target if the input is zero or too small to prevent drift.
            ;; (setq target-shoulder-angle in_progress_target_shoulder_angle))
    ;; (format t "target-shoulder-angle ~a~%" target-shoulder-angle)

    (setq torsoMoveTime 100)
    (moveWaist delta :local torsoMoveTime)
    ;; (unix:usleep (* torsoMoveTime 1000))  ;; no need to sleep since movewaist is a blocking call
    )

(defun collision-check-environment ()

  )

;; this differs from reset-pose in that it sets the orientation of the two hands to be (0 0 0), thus aligning it to an orientaiton ready to grip the wrench.
;; reset-pose sets the two hands to an orientation that is slightly rotated.
(defun reset-task2-pose ()
    (open-hand :larm)
    (open-hand :rarm)
	(send *robot* :reset-pose)
    (send *robot* :larm
          :inverse-kinematics (make-coords :pos (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :pos))) ;; make coords with same hand pos but 0 rot
    (send *robot* :rarm
          :inverse-kinematics (make-coords :pos (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :pos))) ;; make coords with same hand pos but 0 rot    
	(moveRobot 2500)
    (setq in_progress_target_rarm_pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos))
    (setq in_progress_target_rarm_rot (rotation-matrix 0 #f(1 0 0)))
    (setq in_progress_target_shoulder_angle (send *robot* :rarm :shoulder-y :joint-angle))
    )

(defun resetPose ()
	(send *robot* :reset-pose)
	(moveRobot 2500)
    (setq in_progress_target_rarm_pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos))
    (setq in_progress_target_rarm_rot (send (send *robot* :rarm :end-coords :copy-worldcoords) :rot))
    (setq in_progress_target_shoulder_angle (send *robot* :rarm :shoulder-y :joint-angle))
    )

(defun init ()
    (setq rarm-endpos-worldpos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos)) ;; need to use copy-worldcoords as :worldpos returns a reference

    (setq in_progress_target_rarm_pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos))

    (setq in_progress_target_rarm_rot (send (send *robot* :rarm :end-coords :copy-worldcoords) :rot))
    (setq in_progress_target_shoulder_angle (send *robot* :rarm :shoulder-y :joint-angle))

    ;; (setq in_progress_target_torso (send *ri* :move-waist #f(0 0 0) :local))
    (setq gripper-current-pos (send *robot* :rarm-dgripper :joint-angle))    
    
    ;;;;; previous button states ;;;;; 
    ;; initialize to non-null so first time calling joycallback will not crash
    
    (setq prev_LStick_LR 0)
    (setq prev_LStick_UD 0)
    (setq prev_RStick_UD 0)
    (setq prev_RStick_UD 0)
    
    (setq prev_Accl_LR 0)
    (setq prev_Accl_FB 0)
    (setq prev_Accl_UD 0)
    (setq prev_Yaw 0)
    
    (setq prev_DPad_U 0)
    (setq prev_DPad_R 0)
    (setq prev_DPad_D 0)
    (setq prev_DPad_L 0)
        
    (setq prev_L2 0)
    (setq prev_R2 0)
    (setq prev_L1 0)
    (setq prev_R1 0)

    (setq prev_Triangle 0)
    (setq prev_Circle 0)
    (setq prev_Cross 0)
    (setq prev_Square 0)

    (setq prev_Select 0)
    (setq prev_L3 0)
    (setq prev_R3 0)
    (setq prev_Start 0)

    (setq prev_PSButton 0)
)

(defun open-hand (arm)
    (send *ri* :stop-grasp arm)
    (if (equal arm :rarm)
            (send *robot* arm :thumb-r :joint-angle 70)
        (send *robot* arm :thumb-r :joint-angle -70))
    (moverobot 1000)
    )

(defun close-hand (arm)
    (send *ri* :start-grasp arm)
    )

;;; main program starts here ;;;
(matchsimwithrobot)

(init)

(ros::roseus "aero_ps3_joy_mbzirc_teleop")

(setq executing_joycb nil)
(setq isMovedBaseZeroOnce t)
(setq prevMsgTime (instance ros::time :init))

(ros::subscribe "/joy"
                sensor_msgs::joy
                #'callbackJoy 1)

(setq *tfl* (instance ros::transform-listener :init))
(setq *tfb* (instance ros::transform-broadcaster :init))

(ros::advertise "move_base_simple/velocity" geometry_msgs::TwistStamped) 


(setq trajDuration 500)
(print "entering main loop")

(setq *real* t)
(ros::rate 1000)
(setq quit_teleop nil)
(while (and (ros::ok) (not quit_teleop))
    (ros::spin-once)
    (ros::sleep)
    )
