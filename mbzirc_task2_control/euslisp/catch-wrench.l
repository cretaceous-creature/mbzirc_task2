#!/usr/bin/env roseus

(load "package://mbzirc_task2_control/euslisp/catch-wrench-util.l")

(defun full-pick (tar &key (arm :rarm) (auto nil) (ready? nil) (use-feedback? t)
                      (use-torso t) (hand-offset #f(20 -25 0)) (bottom? nil) (feedback-target #f(385 266)) (force-check-thre 1.0))
  (start-picking)
  (set-target tar)
  (send reference-time :now)
  (let ((reach-success nil) ready-state (timeout 100) z-force)
    (gripper-servo-enable)
    (setq ready-state ready?)
    (tagbody start-state
             (unless ready-state
               (format t "next: ready~%")
               (unless auto
                 (read-line))
               (ready-to-reach :use-ri t :move-gripper t)
               (format t "done: ready~%"))
             (format t "next: reach~%")
             (unless auto
               (read-line))
             (setq reach-success (reach-obj :use-ri t :auto auto :use-torso use-torso :timeout timeout
                                            :hand-offset hand-offset :time 4000 :bottom? bottom? :update-time t))
             (format t "done: reach~%")
             (when (not reach-success)
               (setq timeout (* 1.5 timeout))
               (go start-state))
             (setq ready-state nil)
             (when use-feedback?
               (format t "next: centering~%")
               (unless auto
                 (read-line))
               (setq reach-success (fitting-to-wrench-front :use-ri t :auto auto :target-pos feedback-target :time 1000
                                                            :target-accuracy 5 :bottom? bottom?))
               (format t "done: centering~%")
               (when (not reach-success)
                 (return-from full-pick nil))))
    (format t "next: catch~%")
    (get-force-offset)
    (unless auto
      (read-line))
    (move-forward :div 2 :use-ri t :auto auto :use-torso use-torso :use-feedback use-feedback? :time 1500)
    (format t "done: catch~%")
    (when bottom?
      (format t "next: fitting~%")
      (unless auto
        (read-line))
      (fitting-wrench-to-gripper-center :arm arm :use-ri t :use-torso use-torso :vec #f(0 0 15))
      (format t "done: fitting~%"))
    (format t "next: pick~%")
    (unless auto
      (read-line))
    (preset-pick :time 4000)
    (unix::sleep 1)
    (setq z-force (abs (v. (get-offset-force-vector) #f(0 0 1.0))))
    (format t "z-force : ~A~%" z-force)
    (unix::sleep 1)
    (ready-to-reach :use-ri t :move-gripper t :neck-y 0 :time 5000)
    ;;(end-picking)
    (if (< z-force force-check-thre)
            (progn
                (format t "failed to pick wrech!~%")
                (return-from full-pick nil))
        (progn
            (format t "successfully picked wrech!~%")
            (return-from full-pick t)))
    )
  )
