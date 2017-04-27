#!/usr/bin/env roseus

(defun call-base-service-without-result (req_org &key (debug t) (max_vel 1.0) (max_ang 0.3) (use-ri nil))
  (let (req)
    ;;rotate
    (unless (eq (float (send req_org :theta)) 0.0)
      (setq req (copy-object req_org))
      (send req :x 0.0)
      (if (> (/ (send req :theta) (send req :time)) max_ang)
          (progn
            (send req :time (/ (send req :theta) max_ang))
            (if debug
                (format t "time is too short for rotation! overwriten to ~A~%" (send req :time)))))
      (if debug
          (format t "send go pos   :x ~A :theta ~A time: ~A~%" (send req :x) (send req :theta) (send req :time)))
      (when use-ri
        (setq status (ros::service-call "/goblin/gopose" req))
        (if debug
            (format t "status : ~A~%" (send status :status)))
        (unix::sleep (ceiling (* 1.1 (send req :time))))))

    ;;translate
    (unless (eq (float (send req_org :x)) 0.0)
      (setq req (copy-object req_org))
      (if (> (/ (send req :x) (send req :time)) max_vel)
          (progn
            (send req :time (/ (send req :x) max_vel))
            (if debug
                (format t "time is too short for translation! overwriten to ~A~%" (send req :time)))))
      (if debug
          (format t "send go pos   :x ~A :theta ~A time: ~A~%" (send req :x) (send req :theta) (send req :time)))
      (when use-ri
        (setq status (ros::service-call "/goblin/gopose" req))
        (if debug
            (format t "status : ~A~%" (send status :status)))
        (unix::sleep (ceiling (* 1.1 (send req :time))))))))


(defun call-base-service (req_org &key (debug t) (max_vel 1.0) (max_ang 0.3) (use-ri nil))
  (let (req cntr (timeout 100) (rate 10) (*gopose_result_return* (instance geometry_msgs::Pose2D :init)))
    ;;rotate
    (unless (eq (float (send req_org :theta)) 0.0)
      (setq req (copy-object req_org))
      (send req :x 0.0)
      (if (> (/ (send req :theta) (send req :time)) max_ang)
          (progn
            (send req :time (/ (send req :theta) max_ang))
            (if debug
                (format t "time is too short for rotation! overwriten to ~A~%" (send req :time)))))
      (if debug
          (format t "send go pos   :x ~A :theta ~A time: ~A~%" (send req :x) (send req :theta) (send req :time)))
      (when use-ri
        (setq status (ros::service-call "/goblin/gopose" req))
        (if debug
            (format t "status : ~A~%" (send status :status)))
        (unix::sleep (ceiling (send req :time))))
      (unix::sleep 1)
      (setq cntr 0)
      (setq *gopose_result* nil)
      (ros::rate rate)
      (while (null *gopose_result*)
        (ros::spin-once)
        (ros::sleep)
        (if (> (incf cntr) timeout)
            (progn
              (format t "cannot read result!")
              (return-from call-base-service nil))))
      (if debug
          (format t "go pos result :x ~A :theta ~A~%" (send *gopose_result* :x) (send *gopose_result* :theta)))
      (send *gopose_result_return* :x (send *gopose_result* :x))
      (send *gopose_result_return* :theta (send *gopose_result* :theta)))

    ;;translate
    (unless (eq (float (send req_org :x)) 0.0)
      (setq req (copy-object req_org))
      (if (> (/ (send req :x) (send req :time)) max_vel)
          (progn
            (send req :time (/ (send req :x) max_vel))
            (if debug
                (format t "time is too short for translation! overwriten to ~A~%" (send req :time)))))
      (if debug
          (format t "send go pos   :x ~A :theta ~A time: ~A~%" (send req :x) (send req :theta) (send req :time)))
      (when use-ri
        (setq status (ros::service-call "/goblin/gopose" req))
        (if debug
            (format t "status : ~A~%" (send status :status)))
        (unix::sleep (ceiling (send req :time))))
      (unix::sleep 1)
      (setq cntr 0)
      (setq *gopose_result* nil)
      (ros::rate rate)
      (while (null *gopose_result*)
        (ros::spin-once)
        (ros::sleep)
        (if (> (incf cntr) timeout)
            (progn
              (format t "cannot read result!")
              (return-from call-base-service nil))))
      (if debug
          (format t "go pos result :x ~A :theta ~A~%" (send *gopose_result* :x) (send *gopose_result* :theta)))
      (send *gopose_result_return* :x (+ (send *gopose_result_return* :x) (send *gopose_result* :x)))
      (send *gopose_result_return* :theta (+ (send *gopose_result_return* :theta) (send *gopose_result* :theta))))

    ;;return go-pos result
    (return-from call-base-service *gopose_result_return*)
    ))

(defun move-base (target_x target_theta
                           &key (time nil) (use-ri nil) (no-wait nil) (max_vel 1.0) (max_ang 0.3) (x_tolerance 0.1) (theta_tolerance 0.1)
                           (default-speed-x 0.8) (default-speed-rot 0.4)  (max_dist 25) (default-x 3.0)
                           (slow-x-thre 1000) (slow-theta-thre 0.4) (slow-mode-time 3000)
                           (max_loop 3) (use-feedback t) (debug t))
  (if (and (boundp `robot-name) (equal robot-name "hrp2g")) ;; not enable now
      (let* (req *service_result* target_time)
        (if time
            (setq target_time time)
          (progn
            (if (> target_x slow-x-thre)
                (setq target_time_x (/ target_x default-speed-x))
              (setq target_time_x slow-mode-time))
            (if (> target_theta slow-theta-thre)
                (setq target_time_theta (/ target_x default-speed-rot))
              (setq target_time_theta slow-mode-time))
            (setq target_time (max target_time_x target_time_theta))))
        ;;(setq req (instance hrp2g_wheelbase::GoposeRequest :init))
        (send req :x target_x)
        (send req :theta target_theta)
        (send req :time target_time)
        (setq *service_result* (call-base-service req :debug debug :max_vel max_vel :max_ang max_ang :use-ri use-ri))
        (if (null *service_result*)
            (return-from move-base nil))
        (unless use-feedback
          (return-from move-base t))

        ;;feedback from result
        (let* (*gopose_total_result* *gopose_err*
                                     (loop_cntr 0))
          (setq *gopose_total_result* (instance geometry_msgs::Pose2D :init))
          (while (> max_loop loop_cntr)
            (incf loop_cntr)
            (send *gopose_total_result* :x (+ (send *gopose_total_result* :x) (send *service_result* :x)))
            (send *gopose_total_result* :theta (+ (send *gopose_total_result* :theta) (send *service_result* :theta)))
            (setq *gopose_err* (instance geometry_msgs::Pose2D :init))
            (send *gopose_err* :x (- target_x (send *gopose_total_result* :x)))
            (send *gopose_err* :theta (- target_theta (send *gopose_total_result* :theta)))
            (if debug
                (format t "target error  :x ~A :theta ~A~%" (send *gopose_err* :x) (send *gopose_err* :theta)))

            ;; finish when error becomes small enough.
            (when (and (< (abs (send *gopose_err* :x)) x_tolerance) (< (abs (send *gopose_err* :theta)) theta_tolerance))
              (format t "go-pos succeeded.~%")
              (return nil))
            ;; stop when the result is extraordinary.
            (when (or (or (and (not (eq (float (send req :x)) 0.0))
                               (or (> (/ (send *service_result* :x) (send req :x)) 1.5)  (< (/ (send *service_result* :x) (send req :x)) 0.3)))
                          (and (eq (float (send req :x)) 0.0)
                               (> (abs (send *service_result* :x)) 0.3)))
                      (or (and (not (eq (float (send req :theta)) 0.0))
                               (or (> (/ (send *service_result* :theta) (send req :theta)) 1.5)  (< (/ (send *service_result* :theta) (send req :theta)) 0.3)))
                          (and (eq (float (send req :theta)) 0.0)
                               (> (abs (send *service_result* :theta)) 0.3))))
              (format t "Got extraordinary result! Continue?~%")
              (read-line))
            (send req :x (send *gopose_err* :x))
            (send req :theta (send *gopose_err* :theta))
            (send req :time 5.0)
            (if (< (abs (send req :x)) x_tolerance)
                (send req :x 0.0))
            (if (< (abs (send req :theta)) theta_tolerance)
                (send req :theta 0.0))
            (setq *service_result* (call-base-service req :debug debug :max_vel max_vel :max_ang max_ang :use-ri use-ri))
            (if (null *service_result*)
                (return-from move-base nil)))
          (if debug
              (progn (format t "final go-pos result :x ~A :theta ~A~%" (send *gopose_total_result* :x) (send *gopose_total_result* :theta))
                     (format t "final go-pos error  :x ~A :theta ~A~%" (send *gopose_err* :x) (send *gopose_err* :theta))))

          (return-from move-base t)
          ))
    ;; aero
    (progn
        (if time
                (progn
                    (setq target_time_x time)
                    (setq target_time_theta time))
            (progn
                (if (> (abs target_x) slow-x-thre)
                        (setq target_time_x (/ (abs target_x) default-speed-x))
                    (setq target_time_x slow-mode-time))
                (if (> (abs target_theta) slow-theta-thre)
                        (setq target_time_theta (* (/ (abs target_theta) default-speed-rot) 1000))
                    (setq target_time_theta slow-mode-time))))
        (if debug
                (format t "go-pos theta: ~A in ~A~%" target_theta target_time_theta))
        (if debug
                (format t "go-pos x    : ~A in ~A~%" target_x target_time_x))
        (when use-ri
            (unless (equal target_theta 0)
                (format t "go-pos 0 0 ~A ~A~%" target_theta (ceiling target_time_theta))
                (send *ri* :go-pos 0 0 target_theta (ceiling target_time_theta))
                (unless no-wait
                    (format t "wait ~A u secs~%" (ceiling (* target_time_theta 1.2 1000)))
                    (unix::usleep (ceiling (* target_time_theta 1.2 1000))))) ;; wait target_time * 1.2 sec.
            (unless (equal target_x 0)
                (format t "go-pos 0 0 ~A ~A~%" target_x (ceiling target_time_x))
                (send *ri* :go-pos target_x 0 0 (ceiling target_time_x))
                (unless no-wait
                    (format t "wait ~A u secs~%" (ceiling (* target_time_x 1.2 1000)))
                    (unix::usleep (ceiling (* target_time_x 1.2 1000))))) ;; wait target_time * 1.2 sec. 
        ))))

(defun move-base-side (target_y
                           &key (time nil) (use-ri nil) (no-wait nil) (max_vel 1.0)
                           (default-speed-x 0.03) (max_dist 25) (default-x 3.0)
                           (slow-x-thre 100) (slow-theta-thre 0.4) (slow-mode-time 3000)
                           (max_loop 3) (use-feedback t) (debug t))
  (if time
      (setq target_time_y time)
    (progn
      (if (> (abs target_y) slow-x-thre)
          (setq target_time_y (/ (abs target_y) default-speed-x))
        (setq target_time_y slow-mode-time))
      ))
  (if debug
      (format t "go-pos y    : ~A in ~A~%" target_y target_time_y))
  (when use-ri
    (unless (equal target_y 0)
      (send *ri* :go-pos 0 target_y 0 (ceiling target_time_y))
      (unless no-wait
        (unix::usleep (ceiling (* target_time_y 1.2 1000))))) ;; wait target_time * 1.2 sec. 
    ))
