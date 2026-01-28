
(define (domain bomb)
(:predicates (bomb ?x)
(toilet ?x)
(armed ?x)
(clogged ?x))

(:action dunk
:parameters  (?bomb ?toilet)
:precondition (and (bomb ?bomb) (toilet ?toilet)
(not (clogged ?toilet)))
:effect (and (when (armed ?bomb) (not (armed ?bomb)))
(clogged ?toilet)))

(:action flush
:parameters  (?toilet)
:precondition (toilet ?toilet)
:effect (when (clogged ?toilet) (not (clogged ?toilet))))

)

