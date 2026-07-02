;; A larger hand-written scenario: two washroom wings off a corridor.
;; Useful for demonstrating that plan quality (visit ordering) matters —
;; a naive fixed route would cross the corridor four times; the planner
;; finishes each wing before moving on.

(define (problem facility_two_wings)
  (:domain washbot)

  (:objects
    washbot - robot
    dock corridor
    wing_a basin_a commode_a shower_a
    wing_b basin_b commode_b urinal_b - location
  )

  (:init
    (robot_at washbot dock)

    (connected dock corridor)     (connected corridor dock)
    (connected corridor wing_a)   (connected wing_a corridor)
    (connected corridor wing_b)   (connected wing_b corridor)

    (connected wing_a basin_a)    (connected basin_a wing_a)
    (connected wing_a commode_a)  (connected commode_a wing_a)
    (connected wing_a shower_a)   (connected shower_a wing_a)

    (connected wing_b basin_b)    (connected basin_b wing_b)
    (connected wing_b commode_b)  (connected commode_b wing_b)
    (connected wing_b urinal_b)   (connected urinal_b wing_b)

    (dirty basin_a) (dirty commode_a) (dirty shower_a)
    (dirty basin_b) (dirty commode_b) (dirty urinal_b)
  )

  (:goal (and
    (cleaned basin_a) (cleaned commode_a) (cleaned shower_a)
    (cleaned basin_b) (cleaned commode_b) (cleaned urinal_b)
  ))
)
