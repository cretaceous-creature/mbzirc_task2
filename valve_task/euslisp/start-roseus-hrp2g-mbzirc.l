#!/usr/bin/env roseus

(ros::load-ros-manifest "tf")

;;(load "package://hrp2g_utils/euslisp/start_roseus_hrp2g.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2g-interface.l")
(hrp2g-init)
(load "package://hrp2g_utils/euslisp/hrp2g_utils.l")
(objects *hrp2g*)
(matchsimwithrobot)
;;(load "package://hrp2v_dynamixel_controller/euslisp/gripper_controller.l")
;;(load "package://valve_task/euslisp/fit-wrench-to-shaft.l")

;; need to correct rarm end pos since we have a costomized gripper
(ros::roseus "valve_task_start_roseus_hrp2g")
;; (setq *tfl* (instance ros::transform-listener :init))
;; (if (not (send *tfl* :wait-for-transform "RARM_LINK6" "RARM_END_COORDS" (ros::time 0) 0.3))
;;         (progn
;;             (print "ERROR: can't find transformation between RARM_LINK6 and RAMR_END_COORDS")
;;             (exit)
;;             )
;;     )

;(setq rarm-end-coords (send *tfl* :lookup-transform "RARM_LINK6" "RARM_END_COORDS" (ros::time 0)))
;(format t " tf rarm end-coords ~a~%" rarm-end-coords)
;(format t "before correction: hrp2g rarm end-coords ~a~%" (send *hrp2g* :rarm :end-coords))
;(send *hrp2g* :rarm :end-coords :replace-pos (send rarm-end-coords :pos))
;(format t "after correction: hrp2g rarm end-coords ~a~%" (send *hrp2g* :rarm :end-coords))

(defun start-mbzirc-teleop ()
    ;;(load "package://valve_task/euslisp/hrp2g-ps3joy-mbzirc-teleop.l")
    (load "package://valve_task/euslisp/hrp2g-ps3joy-mbzirc-teleop-no-dynamixel.l")
    )

(defun reset-task2-pose ()
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
