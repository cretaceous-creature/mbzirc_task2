#!/usr/bin/env roseus

;; (ros::roseus "align_with_panel")
(load "package://mbzirc_task2_control/euslisp/align-with-panel-util.l")
(load "package://mbzirc_task2_control/euslisp/start-task.l")
;; (start-task)

(defun align-with-panel (&key (target-offset #f(-600 200 0)) (target-rot 0.0) (auto nil) (use-ri t) (timeout 100) (update-time t) (pos_tolerance 20) (rot_tolerance 0.1))
  (start-aligning)
  (let (res)
      (format t "fit rotation~%"
    (setq res (fit-rotation :use-ri use-ri :auto auto  :timeout timeout :target-rot target-rot :update-time update-time :theta_tolerance rot_tolerance))
    (unless res
      ;;(end-aligning)
      (return-from align-with-panel nil))
    (format t "fit translation~%")
    (setq res (fit-translation :use-ri use-ri :auto auto :timeout timeout :target-offset target-offset :update-time update-time :pos_tolerance pos_tolerance))
    (unless res
        ;;(end-aligning)
        (return-from align-with-panel nil))
    (format t "fit rotation for last~%")
    (setq res (fit-rotation :use-ri use-ri :auto auto  :timeout timeout :target-rot target-rot :update-time update-time :theta_tolerance rot_tolerance))
    ;;(end-aligning)
    (return-from align-with-panel res))
  )
