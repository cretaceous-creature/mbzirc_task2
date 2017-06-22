#!/usr/bin/env roseus

(setq *real* t)
(setq *robot* nil)
(setq reference-frame "")
(setq reference-link nil)
(setq robot-name nil)

;; robot should be hrp2g or aero-arm-single-wrist. any other function should be runned after is function.
(defun start-task (&key (robot "aero-arm-single-wrist"))
  (unless   (or (equal robot "hrp2g")
                (equal robot "aero-arm-single-wrist"))
    (format t "invalid robot name!~%")
    (return-from start-picking nil))
  (if (equal robot "hrp2g")
      (progn ;; for hrp2g
        (load "package://hrp2v_dynamixel_controller/euslisp/gripper_controller.l")
        (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2g-interface.l")
        (hrp2g-init)
        (setq *robot* *hrp2g*)
        (send *robot* :angle-vector (send *ri* :state :reference-vector))
        (setq reference-frame "BODY")
        (setq reference-link :body_lk)
        (setq robot-name robot)
        )
    (progn ;; for aero
      (load "package://aeroeus/aero-arm-single-wrist-interface.l")
      (load "package://aero_utils/euslisp/aero-arm-single-wrist-utils.l")
      (aero-init)
      (load-controllers)
      (setq reference-frame "base_link")
      (setq reference-link :base_link_lk)
      (setq robot-name robot)
      (load "package://aero_utils/euslisp/aero-servo-controller.l")
      (load "package://aero_dgripper/euslisp/aero-dgripper-controller.l")
      (setq *real* t)
      (setq *robot* *aero*)
      (send *ri* :go-pos 0 0 0) ;; to lock wheels

      ;;stop image proccessing
      (ros::set-param "/non_ground_box_publisher/run_flag" nil)
      (ros::set-param "/short_range_luminance_homogenizer/run_flag" nil)
      (ros::set-param "/panel_box_publisher/run_flag" nil)
      (ros::set-param "/luminance_homogenizer_panel/run_flag" nil)
      (ros::set-param "/image_clipper_handeye/run_flag" nil)
      ))
  )
