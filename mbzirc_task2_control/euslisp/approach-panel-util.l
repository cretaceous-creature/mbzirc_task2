#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "jsk_pcl_ros")
(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "opencv_apps")

(load "package://mbzirc_task2_control/euslisp/move-base-util.l")

(setq middle-state nil)
(setq *panel_pose* nil)
(setq info_published nil)
(setq *gopose_result* nil)
(setq reference-time (ros::time-now))
(setq detect_published nil)
(setq panel_detected_num nil)

(defun panel-cb (msg)
  (when (ros::time< reference-time (send (send msg :header) :stamp))
    (setq camera_pose (send *tfl* :lookup-transform
                            reference-frame
                            (send msg :header :frame_id)
                            (instance ros::time :init)))
    (setq *body_coords* (send (send *robot* reference-link) :copy-worldcoords))
    (setq camera_world_pose (send *body_coords* :transform (send camera_pose :copy-worldcoords)))
    (setq *panel_pose*
          (send (send camera_world_pose :copy-worldcoords)
                :transform (ros::tf-pose->coords (send msg :pose))))
    (when middle-state
        (setq msg-log msg)
        (setq *panel_pose* (send (send *panel_pose* :copy-worldcoords) :translate
                               (float-vector
                                (- (/ (send (send msg :dimensions) :x) 2.0)) 0 0)))) ;; get front position
  ))

(defun panel-point-cb (msg)
  (when (and (ros::time< reference-time (send (send msg :header) :stamp))
             (null detect_published))
    (setq detect_published t)
    (setq panel_detected_num (length (send msg :points)))
    ))

(defun result-cb (msg)
  (setq *gopose_result* (instance geometry_msgs::Pose2D))
  (send *gopose_result* :x (send msg :x))
  (send *gopose_result* :y (send msg :y))
  (send *gopose_result* :theta (send msg :theta)))

(defun start-long-approach ()
  (setq middle-state nil)
  (ros::unsubscribe "/extract_panel_cluster/output")
  (ros::set-param "/non_ground_box_publisher/run_flag" nil)
  (ros::subscribe "/panel_3d_projector/output" geometry_msgs::PoseStamped #'panel-cb)
)

(defun start-middle-approach ()
  (setq middle-state t)
  (ros::subscribe "/extract_panel_cluster/output" jsk_recognition_msgs::BoundingBox #'panel-cb)
  (ros::unsubscribe "/panel_3d_projector/output")
  (ros::set-param "/non_ground_box_publisher/run_flag" t)
  )

(defun start-go-to-front ()
  (ros::unsubscribe "/extract_panel_cluster/output")
  (ros::set-param "/non_ground_box_publisher/run_flag" nil)
  (ros::unsubscribe "/panel_3d_projector/output")
  (ros::subscribe "/short_range_panel_detector/position" opencv_apps::Point2DArrayStamped #'panel-point-cb)
  (ros::set-param "/short_range_luminance_homogenizer/run_flag" t)
)

(defun start-align ()
  (setq middle-state t)
  (ros::unsubscribe "/panel_3d_projector/output")
  (ros::unsubscribe "/short_range_panel_detector/position")
  (ros::set-param "/short_range_luminance_homogenizer/run_flag" nil)
  (ros::subscribe "/extract_panel_cluster/output" jsk_recognition_msgs::BoundingBox #'panel-cb)
  (ros::set-param "/non_ground_box_publisher/run_flag" t)
  )

(defun end-approach ()
  (ros::unsubscribe "/panel_3d_projector/output")
  (ros::unsubscribe "/short_range_panel_detector/position")
  (ros::set-param "/short_range_luminance_homogenizer/run_flag" nil)
  (ros::unsubscribe "/extract_panel_cluster/output")
  (ros::set-param "/non_ground_box_publisher/run_flag" nil))


(defun approach-panel-onetime (&key (use-ri nil) (auto nil) (target-offset #f(0.0 0.0 0.0)) (keep-dir t) ;;main parameters
                                    (max_dist 15000) (default-x 3000) (default-time 20000) (update-time t)
                                    (x_tolerance 100) (theta_tolerance 0.5))
  (when (equal target-offset #f(0 0 0))
    (format t "Target offset is 0. Move base?~%")
    (read-line))
  (let* ((cntr 0) (timeout 150) *target_pose* target_x target_theta target_time_x)
    (setq *panel_pose* nil)
    (ros::rate 10)
    (when update-time
      (send reference-time :now))
    (while (null *panel_pose*)
        (ros::spin-once)
        (ros::sleep)
        (if (> (incf cntr) timeout)
            (return-from approach-panel-onetime nil)))

    ;;move to panel
    ;;(setq target-offset (scale 1000 target-offset)) ;; roseus to ros
    (setq *target_pose* (send (send *panel_pose* :copy-worldcoords) :translate target-offset :world))
    (setq target-cp (send *target_pose* :copy-worldcoords))
    (setq target_x (min (sqrt (+ (* (elt (send *target_pose* :worldpos) 0) (elt (send *target_pose* :worldpos) 0))
                                 (* (elt (send *target_pose* :worldpos) 1) (elt (send *target_pose* :worldpos) 1))))
                        max_dist))
      (if (< (elt (send *target_pose* :worldpos) 0) 0)
              (setq target_x (- target_x))
          )
    (setq target_theta (atan (/ (elt (send *target_pose* :worldpos) 1) (elt (send *target_pose* :worldpos) 0))))
    (format t "panel found. panel position is x: ~A y: ~A~%go to x: ~A theta: ~A~%"
            (elt (send *panel_pose* :worldpos) 0) (elt (send *panel_pose* :worldpos) 1) target_x target_theta)
    (unless auto
        (read-line))
    (move-base target_x target_theta :use-ri use-ri)
    (when keep-dir
      (move-base 0.0 (- target_theta) :use-ri use-ri))
    (return-from approach-panel-onetime t)
    ))

;;look at panel and rotate base to panel and return t. when panel not detected, return nil.
(defun look-around (&key (use-ri nil) (start-angle -90) (end-angle 90) (divide-num 5) (auto nil)
                         (timeout 150) (update-time t))
    (let (rot-angle (loop_cntr 0) cntr current-angle panel-dir)
        (setq rot-angle (/ (- end-angle start-angle) divide-num))
        (send *robot* :rarm :shoulder-y :joint-angle start-angle)
        (setq current-angle start-angle)
        (unless auto
            (format t "move shoulder-y to ~A~%" current-angle)
            (read-line))
        (moveRobot)
        (while (> divide-num loop_cntr)
            (setq cntr 0)
            (incf loop_cntr)
            (setq *panel_pose* nil)
            (when update-time
                (send reference-time :now))
            (ros::rate 10)
            (while (null *panel_pose*)
                (ros::spin-once)
                (ros::sleep)
                (if (> (incf cntr) timeout)
                        (if (> loop_cntr divide-num)
                                (return-from look-around nil) ;; if panel not found, return nil
                            (return nil))))
            (if (null *panel_pose*)
                    (progn             ;; rotate shoulder
                        (send *robot* :rarm :shoulder-y :joint-angle (+ start-angle (* rot-angle loop_cntr)))
                        (setq current-angle (+ start-angle (* rot-angle loop_cntr)))
                        (unless auto
                            (format t "move shoulder-y to ~A~%" current-angle)
                            (read-line))
                        (moveRobot)
                        )
                (progn
                    ;; detected
                    (setq panel-dir (rad2deg (atan (/ (elt (send *panel_pose* :worldpos) 1) (elt (send *panel_pose* :worldpos) 0)))))
                    (if (< (elt (send *panel_pose* :worldpos) 1) 0)
                            (setq panel-dir (- (abs panel-dir))))
                    (unless auto
                        (format t "panel found. rotate base ~A deg.~%" panel-dir)
                        (read-line))
                    (move-base 0 (deg2rad panel-dir) :use-ri use-ri)
                    (send *robot* :rarm :shoulder-y :joint-angle 0)
                    (moveRobot)
                    (return-from look-around t))))))

(defun check-front (&key (update-time t) (timeout 150))
    (let ((cntr 0))
        (setq detect_published nil)
        (when update-time
            (send reference-time :now))
        (ros::rate 10)
        (while (null detect_published)
            (ros::spin-once)
            (ros::sleep)
            (if (> (incf cntr) timeout)
                    (return-from check-front nil)))
        (if (equal panel_detected_num 1)
                (progn
                    (format t "panel front detected~%")
                    (return-from check-front t))
            (progn
                (format t "panel front not detected~%")
                (return-from check-front nil)))))

(defun prepare-to-align (&key (use-ri nil))
    (move-base 0 (/ PI 2) :use-ri use-ri)
    (if use-ri
            (Task2RecogPose))
    )

(defun go-to-next-side (&key (auto nil) (use-ri t) (offset #f(-1200 500 0)) (panel-size 1000) (rot-dir 0)) ;;offset from panel (right (rot-dir=0)| left (rot-dir=1)) corner in panel coordinates. rot-dir 0:left 1:right
    (let (target_x_1 target_x_2)
        (setq target_x_1 (+ (- (elt offset 0)) (- panel-size (elt offset 1))))
        (setq target_x_2 (+ (- (elt offset 0)) (elt offset 1)))
        (unless auto
            (format t "move base~%")
            (read-line))
        (if (equal rot-dir 0)
            (progn
              (move-base target_x_1 0 :use-ri use-ri)
              (move-base 0 (/ -PI 2) :use-ri use-ri))
          (progn
            (move-base (- target_x_1) 0 :use-ri use-ri)
            (move-base 0 (/ PI 2) :use-ri use-ri)))
        (unless auto
            (format t "move base~%")
            (read-line))
        (if (equal rot-dir 0)
            (move-base target_x_2 0 :use-ri use-ri)
          (move-base (- target_x_2) 0 :use-ri use-ri))))

(defun go-around-panel (&key (auto nil) (use-ri t) (offset-length 1200) (panel-rad 500) (rot-dir 1) (div-num 8)) ;;rot-dir 1:left -1:right
    (let (target_theta target_x go_radius)
      (setq target_theta (/ (/ (* 2 PI) div-num) 2))
      (setq go_radius (+ offset-length panel-rad))
      (setq target_x (* 2 go_radius (sin (abs target_theta))))
      (if (> rot-dir 0)
              (setq target_theta (- target_theta)) ;; for go around left
          (setq target_x (- target_x))) ;; for go around right
      (format t "move base rot: ~A twice , x: ~A~%" target_theta target_x)
      ;;(unless auto
      ;;  (read-line))
      (move-base 0 target_theta :use-ri use-ri)
      (move-base target_x 0 :use-ri use-ri)
      (move-base 0 target_theta :use-ri use-ri)))
