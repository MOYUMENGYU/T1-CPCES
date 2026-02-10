
(define (domain dispose) 

   (:requirements :strips)
   (:predicates (adj ?i ?j) (at ?i) (holding ?o) (obj-at ?o ?i)
                (trash-at ?x) (disposed ?o))
   (:action move
      :parameters (?i ?j)
      :precondition (and (adj ?i ?j) (at ?i))
      :effect (and (not (at ?i)) (at ?j)))
   (:action pickup
      :parameters (?o ?i)
      :precondition (at ?i)
      :effect (when (obj-at ?o ?i) (and (holding ?o) (not (obj-at ?o ?i)))))
   (:action drop
      :parameters (?o ?i)
      :precondition (and (at ?i) (trash-at ?i))
      :effect (when (holding ?o) (and (not (holding ?o)) (disposed ?o)))))

