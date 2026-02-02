(define (problem washroom_problem)
  (:domain washroom)
  (:objects
    r1 - robot
    start basin commode - location
  )
  (:init
    (connected start basin)
    (connected basin start)
    (connected start commode)
    (connected commode start)
    (robot_at r1 start)
  )
  (:goal (and (cleaned basin) (cleaned commode)))
)
