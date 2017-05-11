#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "jsk_pcl_ros")
(ros::load-ros-manifest "opencv_apps")
(ros::load-ros-manifest "jsk_laser")

(setq *target_pose* nil)
(setq *target_pose_orig* nil)
(setq handeye_target nil)
(setq pick-bottom nil)
(setq target_num 0)
(setq laser-reflectance nil)
(setq reference-time (ros::time-now))

(defun pose-top-cb (msg)
  (when (and (ros::time< reference-time (send (send msg :header) :stamp))
             (and (not pick-bottom)
                  (null *target_pose*)))
    (setq camera_pose (send *tfl* :lookup-transform
                            reference-frame
                            (send msg :header :frame_id)
                            (instance ros::time :init)))
    (setq *body_coords* (send (send *robot* reference-link) :copy-worldcoords))
    (setq camera_world_pose (send *body_coords* :transform (send camera_pose :copy-worldcoords)))

    (setq *target_pose*
          (send (send camera_world_pose :copy-worldcoords)
                :transform (ros::tf-pose->coords (elt (send msg :poses) target_num))))
    ))

(defun pose-bottom-cb (msg)
  (when (and (ros::time< reference-time (send (send msg :header) :stamp))
             (and pick-bottom
                  (null *target_pose*)))
    (setq camera_pose (send *tfl* :lookup-transform
                            reference-frame
                            (send msg :header :frame_id)
                            (instance ros::time :init)))
    (setq *body_coords* (send (send *robot* reference-link) :copy-worldcoords))
    (setq camera_world_pose (send *body_coords* :transform (send camera_pose :copy-worldcoords)))

    (setq *target_pose*
          (send (send camera_world_pose :copy-worldcoords)
                :transform (ros::tf-pose->coords (elt (send msg :poses) target_num))))
    ))

(defun point-cb (msg)
  (when (and (ros::time< reference-time (send (send msg :header) :stamp))
             (null handeye_target))
    (let (*handeye_target_array*)
      (setq *handeye_target_array* (send msg :points))
      (if (not (eq (length *handeye_target_array*) 1))
          (progn
            (return-from point-cb nil))
        (setq handeye_target (float-vector (send (car *handeye_target_array*) :x) (send (car *handeye_target_array*) :y))))
   )))

(defun laser-cb (msg)
  (let ((check-point-num 30) (cntr 0) start-point (reflectance-sum 0))
    (setq start-point (- (length (send msg :reflectance)) (floor (/ check-point-num 2))))
    (while (< cntr check-point-num)
      (setq reflectance-sum (+ reflectance-sum (elt (send msg :reflectance) cntr)))
      (incf cntr))
    (setq laser-reflectance (/ reflectance-sum check-point-num))))

(defun ik-planning (target &key (arm :rarm) (rotation-axis t) (use-torso t))
  (if (equal robot-name "hrp2g")
      ;;hrp2g
      (progn
        (send *robot* :fix-leg-to-coords (make-coords))
        (if use-torso
            (send *robot* arm :inverse-kinematics target :rotation-axis rotation-axis :torso t :link-list
                  (send *robot* :link-list (send *robot* arm :end-coords :parent) (car (send *robot* :torso :links))))
          (send *robot* arm :inverse-kinematics target :rotation-axis rotation-axis :torso nil))
        )
    ;;aero-arm
    (if use-torso
        ;; move arm with torso
        (let (torso-target torso-x target-y (upper-offset #f(550 0 200))) ;;upper-offset is ideal position from base to arm-end position in order to solve IK of upper body
          (if (eq rotation-axis nil)
              (progn ;;totation-axis nil
                (if (>= (abs (elt upper-offset 0)) (abs (elt (send target :worldpos) 1)))
                    (progn
                      (if (> (elt (send target :worldpos) 0) 0)
                          (setq torso-x (- (elt (send target :worldpos) 0)
                                           (sqrt (- (expt (elt upper-offset 0) 2)
                                                    (expt (elt (send target :worldpos) 1) 2)))))
                        (setq torso-x (+ (elt (send target :worldpos) 0)
                                         (sqrt (- (expt (elt upper-offset 0) 2)
                                                  (expt (elt (send target :worldpos) 1) 2))))))
                      (setq torso-target (float-vector
                                          torso-x
                                          0.0
                                          (- (elt (send target :worldpos) 2) (elt upper-offset 2)))))
                  (progn
                    (setq torso-target (float-vector
                                        (elt (send target :worldpos) 0)
                                        0.0
                                        (- (elt (send target :worldpos) 2) (elt upper-offset 2)))))))

            (progn ;;totation-axis nil
              (setq target-y (elt (elt (send target :rpy-angle) 0) 0))
              (setq torso-target (float-vector
                                  (+ (elt (send target :worldpos) 0) (/ (elt (send target :worldpos) 1) (tan target-y)))
                                  0.0
                                  (- (elt (send target :worldpos) 2) (elt upper-offset 2))))))
          (format t "target position :  ~A~%" (send target :worldpos))
          (format t "move waist ~A~%" torso-target)
          (send *ri* :move-waist torso-target)
          (setq upper-target (make-coords :pos (v- (send target :worldpos) torso-target)
                                          :rot (send target :worldrot)))
          (format t "upper target position :  ~A~%" (send upper-target :worldpos))
          (send *robot* :fix-leg-to-coords (make-coords))
          (send *robot* arm :inverse-kinematics upper-target :rotation-axis rotation-axis)
          (return-from ik-planning upper-target)
          )
      ;; without torso
      (send *robot* arm :inverse-kinematics target :rotation-axis rotation-axis)
      )))

(defun start-picking ()
  (ros::subscribe "/wrench_3d_projector/pose_top" geometry_msgs::PoseArray #'pose-top-cb)
  (ros::subscribe "/wrench_3d_projector/pose_bottom" geometry_msgs::PoseArray #'pose-bottom-cb)
  (ros::subscribe "/wrench_detector_handeye/position" opencv_apps::Point2DArrayStamped #'point-cb)
  (ros::subscribe "/laser_data" jsk_laser::JskLaser #'laser-cb)
  (ros::set-param "/panel_box_publisher/run_flag" t)
  (ros::set-param "/luminance_homogenizer_panel/run_flag" t)
  (ros::set-param "/image_clipper_handeye/run_flag" t)
)

(defun end-picking ()
  (ros::unsubscribe "/wrench_3d_projector/pose_top")
  (ros::unsubscribe "/wrench_3d_projector/pose_bottom")
  (ros::unsubscribe "/wrench_detector_handeye/position")
  (ros::set-param "/panel_box_publisher/run_flag" nil)
  (ros::set-param "/luminance_homogenizer_panel/run_flag" nil)
  (ros::set-param "/image_clipper_handeye/run_flag" nil))

(defun set-target (tar)
  (setq target_num tar)
  )

(defun ready-to-reach (&key (use-ri nil) (move-gripper nil) (time 7000) (neck-y 0))
    (if (equal robot-name "hrp2g")
            ;;hrp2g
          (progn
              (send *robot* :reset-pose)
          (send *robot* :head :angle-vector (float-vector neck-y 0 0))
          (when use-ri
              (send *ri* :angle-vector (send *robot* :angle-vector) time)
              (send *ri* :wait-interpolation))
          (gripper-servo-enable)
          (when move-gripper
              (gripper-servo-pos 180 5))
          )
      ;;aero-arm
        (progn
            (Task2RecogPose))))

(defun reach-obj (&key (arm :rarm) (rotation-axis t) (timeout 300) (use-ri nil)
                       (use-torso t) (auto nil) (hand-offset #f(0 0 -25)) (bottom? nil)
                       (update-time t) (ignore-rot t) (time 4000))
  ;;(send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (let* (offset-vec pre-offset-vec (cntr 0))
    ;;get target position
    (setq pick-bottom bottom?)
    (setq *target_pose* nil)
    (when update-time
      (send reference-time :now))
    (ros::rate 10)
    (while (null *target_pose*)
      (ros::spin-once)
      (ros::sleep)
      (if (> (incf cntr) timeout)
          (return-from reach-obj nil)))
    (when ignore-rot
        ;; for aero
        (send *target_pose* :replace-rot (rotation-matrix -PI/2 :z)))

    ;; calculate ik-target position
    (if (equal robot-name "hrp2g")
        (progn
          (if bottom?
              (progn
                (setq offset-vec #f(-70 0 0))
                (setq pre-offset-vec #f(-90 -80 -70)))
            (progn
              (setq offset-vec #f(-70 0 0))
              (setq pre-offset-vec #f(-120 0 70))))
          (send *target_pose* :translate hand-offset)
          (setq *target_pose_orig* (send *target_pose* :copy-worldcoords))
          (setq *target_pose_pre* (send (send *target_pose* :copy-worldcoords) :translate offset-vec))
          (setq *target_pose_pre_pre* (send *target_pose_pre* :copy-worldcoords))
          (send *target_pose_pre_pre* :translate pre-offset-vec)

          ;;calulate ik, send motion
          (ik-planning *target_pose_pre_pre* :arm arm :rotation-axis nil :use-torso nil)
          ;;(send *robot* arm :inverse-kinematics *target_pose_pre_pre* :rotation-axis nil :torso nil)
          (when (boundp '*irtviewer*)
              (objects (list *robot* *target_pose_pre_pre*)))
          (unless auto
            (read-line))
          (when use-ri
            (progn
              (send *ri* :angle-vector (send *robot* :angle-vector) time)
              (gripper-servo-pos 300 4)
              (send *ri* :wait-interpolation)
              ))
          (ik-planning *target_pose_pre* :arm arm :rotation-axis rotation-axis :use-torso use-torso)
          (when (boundp '*irtviewer*)
            (objects (list *robot* *target_pose_pre*)))
          (unless auto
            (read-line))
          (when use-ri
            (progn
              (send *ri* :angle-vector (send *robot* :angle-vector) time)
              (unix::sleep 3)
              (gripper-servo-pos 180 2)
              (send *ri* :wait-interpolation)
              ))
          )
      ;;aero-arm
      (progn
        (if bottom?
            (progn
              (setq offset-vec #f(-70 0 0))
              (setq pre-offset-vec #f(-90 0 70)))
          (progn
            (setq offset-vec #f(-120 0 0))
            (setq pre-offset-vec #f(-30 0 90))))
        (send *target_pose* :translate hand-offset)
        (setq *target_pose_orig* (send *target_pose* :copy-worldcoords))
        (setq *target_pose_pre* (send (send *target_pose* :copy-worldcoords) :translate offset-vec))
        (setq *target_pose_pre_pre* (send *target_pose_pre* :copy-worldcoords))
        (send *target_pose_pre_pre* :translate pre-offset-vec)

        ;;calulate ik, send motion
        (send *robot* :reset-task2-pose)
        (setq *target_pose_pre* (ik-planning *target_pose_pre_pre* :arm arm :rotation-axis rotation-axis :use-torso use-torso))
        (send *target_pose_pre* :translate (v- pre-offset-vec))
        
        (when (boundp '*irtviewer*)
          (objects (list *robot* *target_pose_pre_pre*)))
        (unless auto
          (read-line))
        (when use-ri
          (progn
              (send *robot* arm :dgripper :joint-angle 90)
            (send *ri* :angle-vector (send *robot* :angle-vector) time)
            (unix::sleep (ceiling (/ time 1000)))
            ;;(send *ri* :wait-interpolation)
            ))
        (setq *target_pose_orig* (ik-planning *target_pose_pre* :arm arm :rotation-axis rotation-axis :use-torso use-torso))
        (send *target_pose_orig* :translate (v- offset-vec))
        (when (boundp '*irtviewer*)
          (objects (list *robot* *target_pose_pre*)))
        (unless auto
          (read-line))
        (when use-ri
          (progn
            (send *robot* arm :dgripper :joint-angle 0)
            (send *ri* :angle-vector (send *robot* :angle-vector) time)
            (unix::sleep (ceiling (/ time 1000)))
            ;;(send *ri* :wait-interpolation)
            ))
        )
      )))

(defun fitting-to-wrench-front (&key (arm :rarm) (rotation-axis t) (timeout 150) (use-ri nil)
                                     (use-torso t) (auto nil) (target-pos #f(390 285)) (target-accuracy 7) (time 1000)
                                     (max-try 10) (bottom? nil))
  (let (offset-vec (cntr 0) error_vec (try-cntr 0))
    ;;get wrench position
    (setq handeye_target nil)
    (send reference-time :now)
    (ros::rate 10)
    (while (null handeye_target)
      (ros::spin-once)
      (ros::sleep)
      (if (> (incf cntr) timeout)
          (progn (format t "something wrong, return from function~%")
                 (return-from fitting-to-wrench-front nil))))
    (setq error_vec (v- target-pos handeye_target))
    (format t "error vector : ~A ~A~%" (elt error_vec 0) (elt error_vec 1))

    ;;fitting
    (while (and (> (norm error_vec) target-accuracy)
                (< try-cntr max-try))
      ;;(let ((dist-p-param-x 0.25) (dist-p-param-z 0.15) target_vec *ik-target*)
      (let ((dist-p-param-x 0.25) (dist-p-param-z 0.25) target_vec *ik-target*)

        ;;move arm once
        (setq *ik-target* (send (send *robot* arm :end-coords) :copy-worldcoords))
        (setq target_vec (float-vector 0.0 (* dist-p-param-x (elt error_vec 0)) (* dist-p-param-z (elt error_vec 1))))
        (format t "target : ~A ~A ~A~%" (elt target_vec 0) (elt target_vec 1) (elt target_vec 2))
        (send *ik-target* :translate target_vec)
        (ik-planning *ik-target* :arm arm :rotation-axis rotation-axis :use-torso use-torso)
        (when (boundp '*irtviewer*)
          (objects (list *robot* *ik-target*)))
        (unless auto
          (read-line))
        (when use-ri
          (progn
            (send *ri* :angle-vector (send *robot* :angle-vector) time)
            (unix::sleep (ceiling (/ time 1000)))
            ))
        (incf try-cntr)

        ;;update error
        (setq handeye_target nil)
        (send reference-time :now)
        (ros::rate 10)
        (setq cntr 0)
        (while (null handeye_target)
          (ros::spin-once)
          (ros::sleep)
          (if (> (incf cntr) timeout)
              (progn (format t "something wrong, return from function~%")
                     (return-from fitting-to-wrench-front nil))))
        (setq error_vec (v- target-pos handeye_target)))
      (format t "error vector : ~A ~A~%" (elt error_vec 0) (elt error_vec 1)))
    (format t "final error : ~A~%" error_vec)
    (if (> (norm error_vec) target-accuracy)
        (return-from fitting-to-wrench-front nil)
      (progn
        (when bottom?
          (if (equal robot-name "hrp2g")
              (gripper-servo-pos 0 3)
            (gripper-servo-pos 180 3)))
        (return-from fitting-to-wrench-front t)))
    ))

(defun move-forward (&key (div 4) (arm :rarm) (use-ri nil) (auto nil) (use-torso t) (use-feedback t) (time 1500))
  (let (*arm_end* *target_world_dist* *target_dist* *next_target*)
    (dotimes (i div)
      (progn
        ;;(send *robot* :angle-vector (send *ri* :state :potentio-vector))
        (setq *arm_end* (send (send *robot* arm :end-coords) :worldpos))
        (setq *target_world_dist* (v- (send *target_pose_orig* :worldpos) *arm_end*))
        (setq *target_dist* (transform *target_world_dist* (send (send *robot* arm :end-coords) :worldrot)))
        (setq *next_target* (make-coords :pos (send (send (send *robot* arm :end-coords) :copy-worldcoords) :worldpos)
                                         :rot (send *target_pose_orig* :worldrot)))
        (if use-feedback
            (send *next_target* :translate (float-vector (* (/ (elt *target_dist* 0) div) (+ i 1)) 0 0))
          (send *next_target* :translate (float-vector (* (/ (elt *target_dist* 0) div) (+ i 1))
                                                       (* (/ (elt *target_dist* 1) div) (+ i 1))
                                                       (* (/ (elt *target_dist* 2) div) (+ i 1)))))
        (ik-planning *next_target* :arm arm :rotation-axis t :use-torso use-torso)
        ;; (if use-torso
        ;;     (send *robot* arm :inverse-kinematics *next_target* :rotation-axis t :torso t :link-list
        ;;           (send *robot* :link-list (send *robot* arm :end-coords :parent) (car (send *robot* :torso :links))))
        ;;   (send *robot* arm :inverse-kinematics *next_target* :rotation-axis t :torso nil)
        (when (boundp '*irtviewer*)
          (objects (list *robot* *next_target*)))
        (unless auto
          (read-line))
        (when use-ri
            (send *ri* :angle-vector (send *robot* :angle-vector) time)
            ;;(send *ri* :angle-vector (send *robot* :angle-vector) 1000)
            (unix::sleep (ceiling (/ time 1000))))
          ))))

(defun fitting-wrench-to-gripper-center (&key (arm :rarm) (rotation-axis t) (use-ri nil) (use-torso t) (vec #f(0 0 10)) (time 3000))
  (let (*arm_end*)
    ;;(send *robot* :angle-vector (send *ri* :state :potentio-vector))
    (setq *arm_end* (send (send *robot* arm :end-coords) :copy-worldcoords))
    (send *arm_end* :translate vec)
    (ik-planning *arm_end* :arm arm :rotation-axis rotation-axis :use-torso use-torso)
    (when use-ri
      (progn
        (send *ri* :angle-vector (send *robot* :angle-vector) time)
        (unix::sleep (ceiling (/ time 1000)))))
    ;;(send *robot* :angle-vector (send *ri* :state :potentio-vector))
    (setq *arm_end* (send (send *robot* arm :end-coords) :copy-worldcoords))
    (send *arm_end* :translate #f(0 0 -5))
    (ik-planning *arm_end* :arm arm :rotation-axis rotation-axis :use-torso use-torso)
    (when use-ri
        (send *ri* :angle-vector (send *robot* :angle-vector) time)
        (unix::sleep (ceiling (/ time 1000))))
    ))

(defun pick (&key (arm :rarm) (pick-dist (float-vector -80 0 0)) (use-ri nil)
                  (use-torso t) (rotation-axis t) (time 4000) (world? nil))
  (let (*pick_target*)
    ;;(send *robot* :angle-vector (send *ri* :state :potentio-vector))
    (setq *pick_target* (send (send *robot* arm :end-coords) :copy-worldcoords))
    (if world?
        (send *pick_target* :translate pick-dist :world)
      (send *pick_target* :translate pick-dist))
    (ik-planning *pick_target* :arm arm :rotation-axis rotation-axis :use-torso use-torso)
    (when use-ri
      (progn
        (send *ri* :angle-vector (send *robot* :angle-vector) time)
        (unix::sleep (ceiling (/ time 1000.0)))
        ))
  ))

(defun preset-pick (&key (time 3000) (use-torso t))
  (if (equal robot-name "hrp2g")
      (progn
        (pick :use-ri t :pick-dist #f(-100 0 0)    :time time :use-torso use-torso :world? nil)
        (pick :use-ri t :pick-dist #f(-40 -20 -30) :time time :use-torso use-torso)
        (pick :use-ri t :pick-dist #f(-40 -50 -50) :rotation-axis nil :time time :use-torso use-torso))
    (progn
      (pick :use-ri t :time time)
      (pick :use-ri t :time time))
    ))

(defun rot-hand-onetime
  (&key (use-ri nil) (arm :rarm) (use-torso nil) (move-time 2000)
        (move-ang 10) (rot-axis :y) (teleop nil))
  (let (*ik_target* (stable-point-offset (float-vector -40 0 -50)))
        (setq *ik_target* (send (send (send *robot* arm :end-coords) :copy-worldcoords)
                                :translate stable-point-offset))
        (send *ik_target* :rotate (deg2rad move-ang) rot-axis)
        (send *ik_target* :translate (v- stable-point-offset))
        (ik-planning *ik_target* :arm arm :rotation-axis t :use-torso use-torso)
        (when use-ri
          (send *ri* :angle-vector (send *robot* :angle-vector) move-time))
        (when teleop ;;for teleop mode
          (if (equal arm :larm)
              (setq in_progress_target_larm_rot (send (send (send *robot* arm :end-coords) :copy-worldcoords) :rot))
            (setq in_progress_target_rarm_rot (send (send (send *robot* arm :end-coords) :copy-worldcoords) :rot))
        ))))

(defun search-wrench-direction
  (&key (use-ri nil) (arm :rarm) (start-ang 50) (end-ang -50) (div 10)
        (each-move-time 1000) (use-torso t))
  (let ((rot-cnt 0) (cntr 0) (timeout 20) move-ang max-ref max-ref-index max-ref-pose)
    ;; first motion
    (rot-hand-onetime :use-ri use-ri :arm arm :use-torso use-torso :move-ang start-ang)
    (setq move-ang (- (/ (- start-ang end-ang) div)))

    ;; first check
    (setq laser-reflectance nil)
    (ros::rate 10)
    (while (null laser-reflectance)
      (ros::spin-once)
      (ros::sleep)
      (if (> (incf cntr) timeout)
          (progn (format t "something wrong, return from function~%")
                 (return-from search-wrench-direction nil))))
    (setq max-ref laser-reflectance)
    (setq max-ref-index 0)
    (setq max-ref-pose (send *robot* :angle-vector))

    ;; search
    (while (< rot-cnt div)
      (incf rot-cnt)
      (rot-hand-onetime :use-ri use-ri :arm arm :use-torso use-torso :move-ang move-ang :move-time each-move-time)
      (setq laser-reflectance nil)
      (ros::rate 10)
      (while (null laser-reflectance)
        (ros::spin-once)
        (ros::sleep)
        (if (> (incf cntr) timeout)
            (progn (format t "something wrong, return from function~%")
                   (return-from search-wrench-direction nil))))
      (when (> laser-reflectance max-ref)
        (setq max-ref laser-reflectance)
        (setq max-ref-index rot-cnt)
        (setq max-ref-pose (send *robot* :angle-vector)))
      (send *ri* :wait-interpolation)
      )
    (send *robot* :angle-vector max-ref-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (return-from search-wrench-direction t)
    ))
