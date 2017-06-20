#!/usr/bin/env roseus

(ros::roseus "approach_panel")
(load "package://mbzirc_task2_control/euslisp/approach-panel-util.l")
;;(start-task)

(defun long-approach (&key (use-ri t) (auto nil) (start-angle -90) (end-angle 90))
  (start-long-approach)
  (let ((panel_found nil) (approach_res nil) (cntr 0))
    (format t "next: look around~%")
    (unless auto
      (read-line))
    (resetdrivingpose)
    (setq panel_found (look-around :use-ri use-ri :auto auto :start-angle start-angle :end-angle end-angle))
    (format t "done: look around~%")
    (when (null panel_found)
      (end-approach)
      (return-from long-approach nil))

    ;; approach to panel
    (format t "next: approach panel~%")
    (unless auto
      (read-line))
    (setq approach_res (approach-panel-onetime :use-ri use-ri :target-offset #f(-4500 0 0) :auto auto))
    (format t "done: approach panel~%")
    (end-approach)
    (return-from long-approach approach_res)))

(defun middle-approach-look-right (&key (use-ri t) (auto nil))
  (start-middle-approach)
  (let ((panel_found nil) (approach_res nil) (cntr 0))
    (format t "next: look around~%")
    (unless auto
      (read-line))
    ;;(resetdrivingpose)
    (setq panel_found (look-around :use-ri use-ri :auto auto :start-angle -90 :end-angle -90 :divide-num 1))
    (format t "done: look around~%")
    (when (null panel_found)
      (end-approach)
      (return-from middle-approach-look-right nil))

    ;; approach to panel
    (format t "next: approach panel~%")
    (unless auto
      (read-line))
    (setq approach_res (approach-panel-onetime :use-ri use-ri :target-offset #f(-1500 0 0) :auto auto))
    (format t "done: approach panel~%")
    (end-approach)
    (prepare-to-align :use-ri use-ri)
    (return-from middle-approach-look-right approach_res)))

(defun middle-approach (&key (use-ri t) (auto nil))
  (let (res)
    (start-middle-approach)
    (setq res (approach-panel-onetime :use-ri use-ri :target-offset #f(-2000 0 0) :auto auto))
    (unless res
      (end-approach)
      (return-from middle-approach nil))
    (prepare-to-align :use-ri use-ri)
    (setq res (approach-panel-onetime :use-ri use-ri :target-offset #f(0 1200 0) :auto auto))
    (end-approach)
    (return-from middle-approach res)))

(defun short-approach (&key (use-ri t) (auto nil) (rot-dir 1))
  (rough-align :use-ri use-ri :auto auto)
  (go-to-front-of-panel :use-ri use-ri :auto auto :rot-dir rot-dir))

(defun rough-align (&key (use-ri t) (auto nil))
  (let (res)
    (start-align)
    (setq res (approach-panel-onetime :use-ri use-ri :target-offset #f(0 1200 0) :auto auto))
    (end-approach)
    (return-from rough-align res)))

(defun go-to-front-of-panel (&key (auto nil) (use-ri t) (offset-length 1200) (rot-dir 1) (div-num 8) (turn nil))
    (let (check-res (loop_cntr 0) try-num)
      (if turn
              (setq try-num 6)
          (setq try-num div-num))
      (start-go-to-front)
      (tagbody start-state
             (incf loop_cntr)
             (when (> loop_cntr try-num)
                     (format t "check from all position, but could not find front.~%")
                     (end-approach)
                     (return-from go-to-front-of-panel nil))
             (when (and (equal loop_cntr 3) turn)
                 (setq rot-dir (- rot-dir)))
             (format t "next: check-front~%")
             (setq check-res (check-front))
             (format t "done: check-front~%")
             (if check-res
                     (progn
                         (format t "found front of panel.~%")
                         (end-approach)
                         (return-from go-to-front-of-panel t))
                 (progn
                     (format t "next: go-around-panel~%")
                     (unless auto
                         (read-line))
                     (go-around-panel :use-ri use-ri :offset-length offset-length :rot-dir rot-dir :div-num div-num)
                     (format t "done: go-around-panel~%")
                     (go start-state))
                 ))))

;; (when (equal robot-name "hrp2g")
;;   (ros::subscribe "/gopose_results" geometry_msgs::Pose2D #'result-cb))

