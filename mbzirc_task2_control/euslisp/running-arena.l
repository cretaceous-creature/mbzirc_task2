#!/usr/bin/env roseus

(load "package://mbzirc_task2_control/euslisp/move-base-util.l")

(setq search-init-rot 0)

(defun go-to-init-position (&key (x 0) (y 0) (rot 0);; target in mm scale.
                                 (use-ri nil) (speed 1.0))
  (let (first_theta (first_x 3000))
      ;; avoid obstacle
      (if (< y 0)
              (setq first_theta (/ -PI 2))
          (setq first_theta (/ PI 2)))
      (when (not (equal y 0))
         ;; (move-base first_x 0 :use-ri use-ri)
         ;; (setq x (- x first_x))
          (move-base 0 first_theta :use-ri use-ri)
          (move-base (abs y) 0 :use-ri use-ri :time (ceiling (/ (abs y) speed)))
          (move-base 0 (- first_theta) :use-ri use-ri))
      (move-base x 0 :use-ri use-ri :time (ceiling (/ (abs x) speed)))
      (move-base 0 (deg2rad rot) :use-ri use-ri)
      (setq search-init-rot rot))
  (return-from go-to-init-position t))

(defun go-to-next-position (&key (use-ri nil) (speed 1.0) (search-rot 180) (search-dist 10000))
  (let (rot-diff)
    (setq rot-diff (deg2rad (- search-rot search-init-rot)))
    (cond
     ((< rot-diff -PI) (setq rot-diff (+ (* 2 PI) rot-diff)))
     ((> rot-diff PI)  (setq rot-diff (+ (* 2 -PI) rot-diff)))
     (t ()))
    (unless (equal rot-diff 0)
      (move-base 0 rot-diff :use-ri use-ri))
    (setq search-init-rot serach-rot)
    (move-base search-dist 0 :use-ri use-ri :time (ceiling (/ (abs search-dist) speed))))
  (return-from go-to-next-position t))
