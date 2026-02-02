(define (domain washroom)
  (:requirements :strips :typing)
  (:types
    robot location
  )
  (:predicates
    (robot_at ?r - robot ?l - location)
    (connected ?l1 - location ?l2 - location)
    (cleaned ?l - location)
  )

  (:action navigate
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (robot_at ?r ?from) (connected ?from ?to))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )

  (:action clean
    :parameters (?r - robot ?l - location)
    :precondition (and (robot_at ?r ?l))
    :effect (and (cleaned ?l))
  )
)
