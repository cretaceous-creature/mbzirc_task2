#!/usr/bin/env roseus

;; (ros::roseus "align_with_panel")
(load "package://mbzirc_task2_control/euslisp/align-with-panel-util.l")
(load "package://mbzirc_task2_control/euslisp/start-task.l")
;; (start-task)

(defun align-with-panel (&key (target-offset #f(-650 200 0)) (target-rot 0.0) (auto nil) (use-ri t) (timeout 100) (update-time t))
  (start-aligning)
  (let (res)
      (format t "fit rotation~%")
    (setq res (fit-rotation :use-ri use-ri :auto auto  :timeout timeout :target-rot target-rot :update-time update-time))
    (unless res
      (end-aligning)
      (return-from align-with-panel nil))
    (format t "fit translation~%")
    (setq res (fit-translation :use-ri use-ri :auto auto :timeout timeout :target-offset target-offset :update-time update-time))
    (unless res
        (end-aligning)
        (return-from align-with-panel nil))
    (format t "fit rotation for last~%")
    (setq res (fit-rotation :use-ri use-ri :auto auto  :timeout timeout :target-rot target-rot :update-time update-time))
    (end-aligning)
    (return-from align-with-panel res))
  )
