#!/usr/bin/env roseus

(load "package://mbzirc_task2_control/euslisp/move-base-util.l")

;;param
(setq max-side-motion-time 3)
(setq side-motion-onetime-dist 5000)
(setq x-motion-onetime-dist 5000)
(setq first-side-motion-dir 1);; 1 to right, -1 to left

(setq side-motion-count 0)
(setq side-motion-dir first-side-motion-dir)

(defun go-to-init-position (&key (x 0) (y1 0) (y2 1000) ;; target in mm scale.
                                 (use-ri nil) (speed 1.0))
  (let (first_theta (first_x 3000))
    ;; (resetdrivingpose)
      ;; avoid obstacle
      (if (< y1 0)
              (setq first_theta (/ -PI 2))
          (setq first_theta (/ PI 2)))
      (when (not (equal y1 0))
          (move-base first_x 0 :use-ri use-ri)
          (setq x (- x first_x)) ;;check
          (move-base 0 first_theta :use-ri use-ri)
          (move-base (abs y1) 0 :use-ri use-ri :time (ceiling (/ (abs y1) speed)))
          (move-base 0 (- first_theta) :use-ri use-ri))
      (move-base x 0 :use-ri use-ri :time (ceiling (/ (abs x) speed)))
      (move-base 0 (/ -PI 2) :use-ri use-ri)
      (when (not (equal y2 0))
        (move-base (- y2) 0 :use-ri use-ri :time (ceiling (/ (abs y2) speed)))))
  (return-from go-to-init-position t))

(defun side-motion (&key (use-ri nil) (max-side-motion-time 3) (side-motion-dist 7000) (x-motion-dist 3000) (speed 0.6))
  (let (move_x)
    (setq move_x (* side-motion-dir side-motion-onetime-dist))
    (move-base move_x 0 :use-ri use-ri  :time (ceiling (/ (abs move_x) speed)))
    (incf side-motion-count)
    (when (equal side-motion-count max-side-motion-time)
      (move-base 0 (/ -PI 2) :use-ri use-ri)
      (move-base x-motion-onetime-dist 0 :use-ri use-ri :time (ceiling (/ (abs x-motion-onetime-dist) speed)))
      (move-base 0 (/ PI 2) :use-ri use-ri)
      (setq side-motion-dir (- side-motion-dir))
      (setq side-motion-count 0)))
    (return-from side-motion t))
