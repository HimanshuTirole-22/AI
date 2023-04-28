% assignment 1
teacher(marry).
teacher(bunny).
doctor(mili).
doctor(gini).
lives(marry, surat).
lives(bunny, vadodara).
lives(gini, ahmedabad).
lives(mili, rajkot).
earns(marry, fortythousandrupees).
earns(mili, fiftythousandrupees).
earns(bunny, seventythousandrupees).
earns(gini, sixtythousandrupees).

% assignment 2
parent(motilal,javaharlal).
parent(motilal,vijyalakshmi).
parent(motilal,krishna).
parent(swarupmati,javaharlal).
parent(swarupmati,vijyalakshmi).
parent(swarupmati,krishna).
parent(javaharlal,indira).
parent(kamla,indira).
parent(indira,sanjay).
parent(firoz,sanjay).
parent(indira,rajiv).
parent(firoz,rajiv).
parent(sanjay,varun).
parent(menka,varun).
parent(rajiv,rahul).
parent(rajiv,priyanka).
parent(sonia,rahul).
parent(sonia,priyanka).
parent(vijyalakshmi,tara).
parent(vijyalakshmi,lekha).
parent(vijyalakshmi,rita).
parent(ranjit,tara).
parent(ranjit,lekha).
parent(ranjit,rita).

female(kamla).
female(swarupmati).
female(indira).
female(menka).
female(sonia).
female(priyanka).
female(vijyalakshmi).
female(tara).
female(lekha).
female(rita).
female(krishna).

male(motilal).
male(javaharlal).
male(firoz).
male(sanjay).
male(rajiv).
male(varun).
male(rahul).
male(ranjit).

grandparent(X, Y):-
    parent(X, Z),
    parent(Z, Y).

brother(X, Y):-
    parent(Z, X),
    parent(Z, Y),
    female(Z),
    male(X),
    X\=Y.

sister(X, Y):-
    parent(Z, X),
    parent(Z, Y),
    female(Z),
    female(X),
    X\=Y.

uncle(X, Y):-
    male(X),
    parent(Z, X),
    grandparent(Z, Y),
    not(parent(X, Y)).

aunt(X, Y):-
    female(X),
    parent(Z, X),
    grandparent(Z, Y),
    not(parent(X, Y));
    wife(X, Z),
    uncle(Z, Y).

mother(X, Y):-
    parent(X, Y),
    female(X).

father(X, Y):-
    parent(X, Y),
    male(X).

wife(X, Y):-
    parent(X, Z),
    parent(Y, Z),
    female(X),
    male(Y).

child(X, Y):-
    parent(Y, X).

son(X, Y):-
    parent(Y, X),
    male(X).

daughter(X, Y):-
    parent(Y, X),
    female(X).

% assignment 3
%1
printName:-
    write('Enter name'),
    read(X),
    write(X).

%2
operate(X,A,B,R):-X=:=1, R is A+B.
operate(X,A,B,R):-X=:=2, R is A-B.
operate(X,A,B,R):-X=:=3, R is A*B.
operate(X,A,B,R):-X=:=4, R is A/B.

calculator:-
    write('Enter 2 numbers'),
    read(A),
    read(B),
    write('Enter 1. Add 2. Subtract 3. Multiply 4. Divide'),
    read(X),
    operate(X, A, B, R),
    write(R).

%3
max(P,Q,R):-P>Q,P>R,write('Larger number is '),write(P).
max(P,Q,R):-P>Q,P=R,write('Larger number is '),write(P).
max(P,Q,R):-P>R,P=Q,write('Larger number is '),write(P).
max(P,Q,R):-Q>P,Q>R,write('Larger number is '),write(Q).
max(P,Q,R):-Q>P,Q=R,write('Larger number is '),write(Q).
max(P,Q,R):-Q>R,Q=P,write('Larger number is '),write(Q).
max(P,Q,R):-R>P,R>Q,write('Larger number is '),write(R).
max(P,Q,R):-R>P,R=Q,write('Larger number is '),write(R).
max(P,Q,R):-R>Q,R=P,write('Larger number is '),write(R).
max(P,Q,R):-P=Q,P=R,write('All Equal').

min(P,Q,R):-P<Q,P<R,write('Smaller number is '),write(P).
min(P,Q,R):-P<Q,P=R,write('Smaller number is '),write(P).
min(P,Q,R):-P<R,P=Q,write('Smaller number is '),write(P).
min(P,Q,R):-Q<P,Q<R,write('Smaller number is '),write(Q).
min(P,Q,R):-Q<P,Q=R,write('Smaller number is '),write(Q).
min(P,Q,R):-Q<R,Q=P,write('Smaller number is '),write(Q).
min(P,Q,R):-R<P,R<Q,write('Smaller number is '),write(R).
min(P,Q,R):-R<P,R=Q,write('Smaller number is '),write(R).
min(P,Q,R):-R<Q,R=P,write('Smaller number is '),write(R).
min(P,Q,R):-P=Q,P=R,write('All Equal').

%4
maxMinNum:-
    write('Enter 3 numbers'),
    read(X),
    read(Y),
    read(Z),
    max(X,Y,Z),
    nl,
    min(X,Y,Z).

days(X):- X mod 2 =:= 0, write('Monday, Wednesday, Friday & Sunday').
days(X):-X mod 2 =:= 1, write('Tuesday, Thursday, Saturday & Sunday').
vehicle:-
    write('Enter number plate number'),
    read(X),
    days(X).

%assignment 5
% for finding the length
list_length([],0).
list_length([_|TAIL],N) :- list_length(TAIL,N1), N is N1 + 1.

% to find the member of list
list_member(X,[X|_]).
list_member(X,[_|TAIL]) :- list_member(X,TAIL).


% to find the sum of list
list_sum([],0).
list_sum([Head|Tail], Sum) :-
   list_sum(Tail,SumTemp),
   Sum is Head + SumTemp.

%find last element of list
lastElement([Head]) :- write(Head).
lastElement([_|Tail]):-
    lastElement(Tail).


% reverse of list
list_concat([],L,L).
list_concat([X1|L1],L2,[X1|L3]) :- list_concat(L1,L2,L3).

list_rev([],[]).
list_rev([Head|Tail],Reversed) :-
   list_rev(Tail, RevTail),list_concat(RevTail, [Head],Reversed).

