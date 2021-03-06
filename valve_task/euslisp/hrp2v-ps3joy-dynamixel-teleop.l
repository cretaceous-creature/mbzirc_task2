#!/usr/bin/env roseus

(load "package://hrp2v_utils/euslisp/hrp2w_mbzirc_utils.l")
;;(load "package://vmax_controller/euslisp/vmax_basic_operation.l")
(load "package://valve_task/euslisp/fit-wrench-to-shaft.l")

(ros::load-ros-manifest "joint_angles_tracker")
(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "std_srvs")

(setq *robot* *hrp2w_mbzirc*)

(setq isMoveRobot nil)


(defun callbackJoy (joyState)
    ;; (print "in callbackJoy")

    ;; euslisp is calling this mulitple time in different threads before the previous callbacks returns.
    ;; Need to prevent problem with this bug!!
    (if executing_joycb
            (return-from callbackJoy)
        )
    (setq executing_joycb t)
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
    (setq stick_thre 0.02)
    
    
    (setq mode nil)
    (cond
     ((and (< L1 trigger_thre) (> L2 trigger_thre) (> R1 trigger_thre) (> R2 trigger_thre))
      (setq mode "base_mode"))
     ((and (< L2 trigger_thre) (> L1 trigger_thre) (> R1 trigger_thre) (> R2 trigger_thre))
      (setq mode "larm_mode"))
     ((and (< R2 trigger_thre) (> L1 trigger_thre) (> L2 trigger_thre) (> R1 trigger_thre))
      (setq mode "rarm_mode"))
     ((and (< L2 trigger_thre) (< R2 trigger_thre) (> L1 trigger_thre) (> R1 trigger_thre))
      (setq mode "dual_arm_mode"))
     ((and (< R1 trigger_thre) (> L1 trigger_thre) (> L2 trigger_thre) (> R2 trigger_thre))
      (setq mode "head_mode"))
     )
    
    ;; vmax won't stop moving once a non-zero twist command is sent.
    ;; need to send zero velocity command once L1 button is release or vmax will never stop.
    (if (and (< prev_L1 trigger_thre) (>= L1 trigger_thre)) 
            (cmd_vel 0 0 0))

    (cond
     ((equal mode "base_mode")
      (progn          
          (let* ((linear-vel-gain 0.15) (angular-vel-gain (deg2rad 40))
                 (dx (* linear-vel-gain LStick_UD)) (dy (* linear-vel-gain LStick_LR)) (dth (* angular-vel-gain RStick_LR))
                 )
              ;; (format t "L1 button: ~A~%" L1-flag)
              ;; (format t "X: ~A  Y: ~A  TH: ~A~%" x-flag y-flag th-flag)
              (format t "dx: ~A  dy: ~A  dth: ~A~%" (* (abs L1) dx) (* (abs L1) dy) (* (abs L1) dth))
              (cmd_vel (* (abs L1) dx) (* (abs L1) dy) (* (abs L1) dth))
              )
          )
      )
     ((or (equal mode "larm_mode") (equal mode "rarm_mode"))
      (progn
          ;; (print "arm_mode")
          (if (equal mode "larm_mode")
                  (setq arm :larm)
              (setq arm :rarm))
          (if (equal mode "rarm_mode")
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
                                  (gripper-servo-pos 180 3)
                                  (print "gripper-servo-enable")
                                  )
                          )
                      (if (< Triangle 0)
                              (progn
                                  (print "to be implemented")
                                  )
                          )
                      (if (< Cross 0)
                              (progn
                                  (print "to be implemented")
                                  )
                          )
                      )
              )
          (if (equal mode "larm_mode")
                  (progn
                      (if (and (< prev_Circle button_thre) (> Circle button_thre))
                              (send *ri* :stop-grasp arm))
                      (if (and (< prev_Square button_thre) (> Square button_thre))
                              (send *ri* :start-grasp arm))
                      (if (< Triangle 0)
                              (progn
                                  (send *ri* :stop-grasp arm)
                                  (setq moveThumbGain 30)
                                  (move-thumb arm (* moveThumbGain (abs Triangle)))
                                  )
                          )
                      (if (< Cross 0)
                              (progn
                                  (setq moveThumbGain 30)
                                  (move-thumb arm (* moveThumbGain Cross))
                                  )
                          )
                      )
              )
          (if (or (> (abs RStick_UD) stick_thre) (> (abs RStick_LR) stick_thre) (> (abs LStick_UD) stick_thre))
                  (progn
                      (setq moveArmGain 50)
                      (move-arm arm (scale moveArmGain (float-vector RStick_UD RStick_LR LStick_UD)))
                      )
              )
          (if (and (equal mode "rarm_mode") (< DPad_U button_thre))
                  (progn
                      (move-arm-to-start-pose)
                      (setq in_progress_target_larm_pos (send (send *robot* :larm :end-coords :copy-worldcoords) :pos))
                      (setq in_progress_target_rarm_pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos))
                      (setq in_progress_target_larm_rot (rotation-matrix 0 #f(1 0 0)))
                      (setq in_progress_target_rarm_rot (rotation-matrix 0 #f(1 0 0)))
                      )
              )
          (if (and (equal mode "rarm_mode") (< DPad_D button_thre))
                  (progn
                      (if (boundp `arm-delta)
                              (setq arm-delta (fit-wrench-to-shaft arm-delta))
                          (setq arm-delta (fit-wrench-to-shaft))
                          )
                      (setq in_progress_target_larm_pos (send (send *robot* :larm :end-coords :copy-worldcoords) :pos))
                      (setq in_progress_target_rarm_pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos))
                      (setq in_progress_target_larm_rot (rotation-matrix 0 #f(1 0 0)))
                      (setq in_progress_target_rarm_rot (rotation-matrix 0 #f(1 0 0)))
                      )
              )
          )
      )
     ((equal mode "dual_arm_mode")
      (progn
          ;; (print "dual_arm_mode")
          (if (< DPad_L button_thre)
                  (progn
                      (print "reset pose")
                      (reset-task2-pose)
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
                      (move-head (scale moveHeadGain (float-vector LStick_LR (- RStick_UD))))
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

    (setq executing_joycb nil)
    )

(Defun Move-arm (arm delta)
    
    (format t "delta ~a~%" delta)
    (if (equal arm :larm)
            (progn 
                (setq in_progress_target_arm_pos in_progress_target_larm_pos)
                (setq in_progress_target_arm_rot in_progress_target_larm_rot))
        (progn         
            (setq in_progress_target_arm_pos in_progress_target_rarm_pos)
            (setq in_progress_target_arm_rot in_progress_target_rarm_rot))
        )
        
    (format t "in_progress_target ~a~%" in_progress_target_arm_pos)

    (setq tmp-angle-vec (send *robot* :angle-vector)) ;; need to save the robot angle vector before calling matchsimwithrobot first to prevet drifting
    (matchsimwithrobot) 
    (setq current-arm-pos (send (send *robot* arm :end-coords :copy-worldcoords) :pos)) ;; need to use copy-worldcoords as :worldpos returns a reference
    (setq target-arm-pos (v+ current-arm-pos delta))
    (format t "target-arm-pos ~a~%" target-arm-pos)
    (setq target-pos-diff (v- in_progress_target_arm_pos target-arm-pos))
    (format t "target-pos-diff ~a~%" target-pos-diff)
    (send *robot* :angle-vector tmp-angle-vec) ;; done calculations, restore model in sim
    
    ;; check to see that the new target is in the same direction as delta relative to the in progress target to prevent backlash
    (setq move-arm-thre 0.02)
    (setq i 0)
    (while (< i 3)
        (if (or (< 0 (* (elt target-pos-diff i) (elt delta i))) ;; avoid sending target that is closer to what has already been sent.
                (< (abs (elt delta i)) move-arm-thre)) ;; don't change the component of the target if the input is zero or too small to prevent drift.
                (setf (aref target-arm-pos i) (aref in_progress_target_arm_pos i)))
        (setq i (+ i 1))
        )
    (format t "target-arm-pos ~a~%" target-arm-pos)

    (send *robot* arm :inverse-kinematics (make-coords :pos target-arm-pos :rot in_progress_target_arm_rot)
          :thre 0.1
          :rthre 0.1)
    (moverobot 100)
    
    (if (equal arm :larm)
            (setq in_progress_target_larm_pos target-arm-pos)
        (setq in_progress_target_rarm_pos target-arm-pos))
    )

(defun move-thumb (arm delta)
    
    ;; close/open directions are opposite for left and right hands
    ;; flip the sign for larm so that positive delta is opening hand for both arms
    (if (equal arm :larm) 
            (setq delta (- delta)))

    (format t "delta ~a~%" delta)
    (if (equal arm :larm)
            (setq in_progress_target_thumb_angle in_progress_target_lthumb_angle)
        (setq in_progress_target_thumb_angle in_progress_target_rthumb_angle)
        )
            

    (format t "in_progress_target ~a~%" in_progress_target_thumb_angle)

    (setq tmp-angle-vec (send *robot* :angle-vector)) ;; need to save the robot angle vector before calling matchsimwithrobot first to prevet drifting
    (matchsimwithrobot) 
    (setq current-thumb-angle (send *robot* arm :thumb-r :joint-angle))
    (setq target-thumb-angle (+ current-thumb-angle delta))
    (format t "target-thumb-angle ~a~%" target-thumb-angle)
    (setq target-thumb-angle-diff (- in_progress_target_thumb_angle target-thumb-angle))
    (format t "target-thumb-angle-diff ~a~%" target-thumb-angle-diff)
    (send *robot* :angle-vector tmp-angle-vec) ;; done calculations, restore model in sim
    
    ;; check to see that the new target is in the same direction as delta relative to the in progress target to prevent backlash
    (setq move-thumb-thre 0.02)
    (setq i 0)

    (if (or (< 0 (* target-thumb-angle-diff delta)) ;; avoid sending target that is closer to what has already been sent.
            (< (abs delta) move-thumb-thre)) ;; don't change the component of the target if the input is zero or too small to prevent drift.
            (setq target-thumb-angle in_progress_target_thumb_angle))
    (format t "target-thumb-angle ~a~%" target-thumb-angle)

    (send *robot* arm :thumb-r :joint-angle target-thumb-angle)
    (moverobot 100)

    (if (equal arm :larm)
            (setq in_progress_target_lthumb_angle target-thumb-angle)
        (setq in_progress_target_rthumb_angle target-thumb-angle))
    )


(defun move-head (delta)
    
    (format t "delta ~a~%" delta)
    (format t "in_progress_target ~a~%" in_progress_target_neck_angles)

    (setq tmp-angle-vec (send *robot* :angle-vector)) ;; need to save the robot angle vector before calling matchsimwithrobot first to prevet drifting
    (matchsimwithrobot) 
    (setq current-neck-angles
          (float-vector
           (send *robot* :head :neck-y :joint-angle)
           (send *robot* :head :neck-p :joint-angle)
           )
          )
    (setq target-neck-angles (v+ current-neck-angles delta))
    (format t "target-neck-angles ~a~%" target-neck-angles)
    (setq target-neck-angles-diff (v- in_progress_target_neck_angles target-neck-angles))
    (format t "target-neck-angles-diff ~a~%" target-neck-angles-diff)
    (send *robot* :angle-vector tmp-angle-vec) ;; done calculations, restore model in sim
    
    ;; check to see that the new target is in the same direction as delta relative to the in progress target to prevent backlash
    (setq move-neck-thre 0.02)
    (setq i 0)
    (while (< i 2)
        (if (or (< 0 (* (elt target-neck-angles-diff i) (elt delta i))) ;; avoid sending target that is closer to what has already been sent.
                (< (abs (elt delta i)) move-neck-thre)) ;; don't change the component of the target if the input is zero or too small to prevent drift.
                (setf (aref target-neck-angles i) (aref in_progress_target_neck_angles i)))
        (setq i (+ i 1))
        )
    (format t "target-neck-angles ~a~%" target-neck-angles)

    (send *robot* :head :neck-y :joint-angle (elt target-neck-angles 0))
    (send *robot* :head :neck-p :joint-angle (elt target-neck-angles 1))
    (moverobot 100)
    
    (setq in_progress_target_neck_angles target-neck-angles)

    )

    
(defun movehand (theta)
    (setq normal_vector #f(1 0 0)) ;; may want to change this to the normal vector of the panel later
    (setq zaxis #f(0 0 1))
    (setq arm :larm)
    
    (setq startpos (send (send *robot* arm :end-coords :copy-worldcoords) :pos)) ;; need to use copy-worldcoords as :worldpos returns a reference
    (setq target startpos)
    ;; Compute hand orientation. Only allow 1 Dof Rotation about xaxis. 
    (setq panel_normal_orientation (make-matrix 3 3 (list normal_vector (v* zaxis normal_vector) zaxis)))
    (setq rot (m* (rotation-matrix theta #f(-1 0 0)) panel_normal_orientation))
    (send *robot* arm :inverse-kinematics (make-coords :pos target :rot rot)
          :thre 0.1
          ;; :translation-axis nil
          ;; :collision-avoidance-link-pair nil
          :debug-view t
         )

    ;; (send *robot* arm :inverse-kinematics (make-coords :pos (send *robot* arm :end-coords :worldpos) :rot rot)
          ;; :thre 1000
          ;; :translation-axis nil
          ;; :collision-avoidance-link-pair nil
          ;; :debug-view t
         ;; )
    ;; (if (boundp '*irtviewer*)
		;; (send *irtviewer* :draw-objects))
    (setq endpos (send (send *robot* arm :end-coords :copy-worldcoords) :pos))
    (print startpos)
    (print endpos)
    (print (v- endpos target))
    (moverobot 1000)
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
    (setq in_progress_target_larm_pos (send (send *robot* :larm :end-coords :copy-worldcoords) :pos))
    (setq in_progress_target_rarm_pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos))
    (setq in_progress_target_larm_rot (rotation-matrix 0 #f(1 0 0)))
    (setq in_progress_target_rarm_rot (rotation-matrix 0 #f(1 0 0)))
    (setq in_progress_target_lthumb_angle (send *robot* :larm :thumb-r :joint-angle))
    (setq in_progress_target_rthumb_angle (send *robot* :rarm :thumb-r :joint-angle))
    (setq in_progress_target_neck_angles
          (float-vector
           (send *robot* :head :neck-y :joint-angle)
           (send *robot* :head :neck-p :joint-angle)
           )
          )
    )

(defun resetPose ()
    (open-hand :larm)
    (open-hand :rarm)
	(send *robot* :reset-pose)
	(moveRobot 2500)
    (setq in_progress_target_larm_pos (send (send *robot* :larm :end-coords :copy-worldcoords) :pos))
    (setq in_progress_target_rarm_pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos))
    (setq in_progress_target_larm_rot (send (send *robot* :larm :end-coords :copy-worldcoords) :rot))
    (setq in_progress_target_rarm_rot (send (send *robot* :rarm :end-coords :copy-worldcoords) :rot))
    (setq in_progress_target_lthumb_angle (send *robot* :larm :thumb-r :joint-angle))
    (setq in_progress_target_rthumb_angle (send *robot* :rarm :thumb-r :joint-angle))
    (setq in_progress_target_neck_angles
          (float-vector
           (send *robot* :head :neck-y :joint-angle)
           (send *robot* :head :neck-p :joint-angle)
           )
          )
    )

(defun init ()
    (setq larm-endpos-worldpos (send (send *robot* :larm :end-coords :copy-worldcoords) :pos)) ;; need to use copy-worldcoords as :worldpos returns a reference
    (setq rarm-endpos-worldpos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos)) ;; need to use copy-worldcoords as :worldpos returns a reference

    (setq in_progress_target_larm_pos (send (send *robot* :larm :end-coords :copy-worldcoords) :pos))
    (setq in_progress_target_rarm_pos (send (send *robot* :rarm :end-coords :copy-worldcoords) :pos))

    (setq in_progress_target_lthumb_angle (send *robot* :larm :thumb-r :joint-angle))
    (setq in_progress_target_rthumb_angle (send *robot* :rarm :thumb-r :joint-angle))

    (setq in_progress_target_larm_rot (send (send *robot* :larm :end-coords :copy-worldcoords) :rot))
    (setq in_progress_target_rarm_rot (send (send *robot* :rarm :end-coords :copy-worldcoords) :rot))

    (setq in_progress_target_neck_angles
          (float-vector
           (send *robot* :head :neck-y :joint-angle)
           (send *robot* :head :neck-p :joint-angle)
           )
          )
    
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

(ros::roseus "hrp2_tracker_teleop")

;; (ros::subscribe "/imu/data2"
				;; sensor_msgs::Imu
				;; #'callbackImu 1)

;; (ros::subscribe "/wiimote/state"
                ;; wiimote::State
                ;; #'callbackWiimote 100)

(ros::subscribe "/joy"
                sensor_msgs::Joy
                #'callbackJoy 1)

(setq *tfl* (instance ros::transform-listener :init))
(setq *tfb* (instance ros::transform-broadcaster :init))

(setq trajDuration 500)
(print "entering main loop")

(setq *real* t)
(ros::rate 1000)
(setq quit_teleop nil)
(setq executing_joycb nil)
(while (and (ros::ok) (not quit_teleop))
    (ros::spin-once)
    (ros::sleep)
    )
