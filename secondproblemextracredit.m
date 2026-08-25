% Solve two simultaneous equations symbolically

syms x y

eq1 = 2*x + y == 7;
eq2 = x - y == 2;

solution = solve([eq1, eq2], [x, y]) % solution

