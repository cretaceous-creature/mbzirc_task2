#!/usr/bin/env roseus

(ros::load-ros-manifest "geometry_msgs")
(load "package://mbzirc_task2_control/euslisp/move-base-util.l")

(setq reference-time (ros::time-now))
(setq panel-corner nil)
(setq base-frame "ground")

(defun pose-roi-cb (msg)
  (when (ros::time< reference-time (send (send msg :header) :stamp))
      (let (rot camera_rot)
          (setq camera_pose (send *tfl* :lookup-transform
                                  base-frame
                                  (send msg :header :frame_id)
                                  (instance ros::time :init)))

          (setq camera_world_pose (send (make-coords) :transform (send camera_pose :copy-worldcoords)))
          ;; for dealing with bug temporary
          (setq camera_rot (acos (v. #f(1 0 0) (send camera_pose :x-axis))))
          (if (< (elt (send camera_pose :x-axis) 1) 0)
                  (setq camera_rot (- camera_rot)))
          (send camera_world_pose :rotate (* 2 camera_rot) :z)
          (setq panel-local (ros::tf-pose->coords (elt (send msg :poses) 0)))

          ;; for dealing with bug temporary
          (setq panel-local  (ros::tf-pose->coords (elt (send msg :poses) 0)))
          (setq panel-corner
                (send (send camera_pose :copy-worldcoords)
                      :transform (ros::tf-pose->coords (elt (send msg :poses) 0))))
          (setq panel-cp (send panel-corner :copy-worldcoords))
          (setq rot (acos (v. #f(1 0 0) (send panel-cp :x-axis))))
          (if (< (elt (send panel-corner :x-axis) 1) 0)
                  (setq rot (- rot)))
          (send panel-corner :rotate (* 2 rot) :z)
          )))

(defun start-aligning ()
  (ros::subscribe "/panel_roi_generator/debug_output" geometry_msgs::PoseArray #'pose-roi-cb)
  (ros::set-param "/panel_box_publisher/run_flag" t))

(defun end-aligning ()
  (ros::unsubscribe "/panel_roi_generator/debug_output")
  (ros::set-param "/panel_box_publisher/run_flag" nil))

(defun fit-rotation (&key (auto nil) (timeout 100) (update-time t) (use-ri nil) (use-feedback t)
                          (align-axis :x) ;; axis to be parallel with panel (align-axis will be aligned with y axis of panel).
                          (theta_tolerance 0.1) (target-rot 0) (max_loop 6))
  (let (corner-axis error-rot (loop_cntr 0) cntr)
    (while (> max_loop loop_cntr)
      (setq cntr 0)
      (incf loop_cntr)
      (setq panel-corner nil)
      (when update-time
        (send reference-time :now))
      (ros::rate 10)
      (while (null panel-corner)
        (ros::spin-once)
        (ros::sleep)
        (if (> (incf cntr) timeout)
            (return-from fit-rotation nil)))

      ;; calculate thera error
      (if (equal align-axis :x)
          (setq corner-axis (send panel-corner :y-axis)) ;; x of robot and y of panel will be parallel
        (setq corner-axis (send panel-corner :x-axis))) ;; x of robot and x of panel will be parallel
      (format t "corner axis : ~A~%" corner-axis)
      (setq error-rot (acos (v. #f(1 0 0) corner-axis)))
      (when (< (v. #f(0 1 0) corner-axis) 0)
          (setq error-rot (- error-rot)))
      (setq error-rot (+ error-rot target-rot))
      (when (< (abs error-rot) theta_tolerance)
        (format t "fitting rotation succeeded.~%")
        (return-from fit-rotation t))

      (format t "rotate base ~A~%" error-rot)
      (unless auto
        (read-line))
      (move-base 0 error-rot :use-ri use-ri)
      (when (not use-feedback)
        (return-from fit-rotation t))
      )))

(defun fit-translation (&key (target-offset #f(-650 200 0)) ;; target-offset is offset from panel in local coordinates of panel
                             ;; (auto nil) (use-ri t) (use-feedback t) (keep-dir t) (world nil)
                             (auto nil) (use-ri t) (use-feedback t) (world nil)
                             (timeout 100) (update-time t) (pos_tolerance 50) (max_loop 6))
  (let (error-pos (loop_cntr 0) target_x target_theta cntr)
    (while (> max_loop loop_cntr)
      (incf loop_cntr)
      (setq panel-corner nil)
      (setq cntr 0)
      (when update-time
        (send reference-time :now))
      (ros::rate 10)
      (while (null panel-corner)
        (ros::spin-once)
        (ros::sleep)
        (if (> (incf cntr) timeout)
            (return-from fit-translation nil)))

      ;; calculate position error
      (if world
              (setq error-pos (send (send (send (send panel-corner :copy-worldcoords) :translate (v- target-offset) :world) :copy-worldcoords) :worldpos))
            (setq error-pos (send (send (send (send panel-corner :copy-worldcoords) :translate (v- target-offset)) :copy-worldcoords) :worldpos)))
      (setf (elt error-pos 2) 0)
      (when (< (norm error-pos) pos_tolerance)
        (format t "fitting translation succeeded. final error is ~A~%" (norm error-pos))
        (return-from fit-translation t))

      (format t "move base x: ~A y: ~A~%" (elt error-pos 0) (elt error-pos 1))
      ;; (setq target_x (sqrt (+ (* (elt error-pos 0) (elt error-pos 0))
      ;;                         (* (elt error-pos 1) (elt error-pos 1)))))
      ;; (if (< (elt error-pos 0) 0)
      ;;         (setq target_x (- target_x))
      ;;     )
      ;; (setq target_theta (atan (/ (elt error-pos 1) (elt error-pos 0))))
      ;; (format t "move base target_x: ~A target_theta: ~A~%" target_x target_theta)
      ;; (unless auto
      ;;     (read-line))
      ;; (move-base target_x target_theta :use-ri use-ri)
      ;; (when keep-dir
      ;;   (move-base 0.0 (- target_theta) :use-ri use-ri))
      (unless auto
          (read-line))
      (move-base (elt error-pos 0) 0 :use-ri use-ri)
      (move-base-side (elt error-pos 1) :use-ri use-ri)
      (unix::sleep 3)
      (when (not use-feedback)
          (return-from fit-translation t)
          ))
    ))
