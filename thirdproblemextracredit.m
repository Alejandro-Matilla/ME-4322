% Ask the user for two numbers
firstNumber = input('Enter the first number: ');
secondNumber = input('Enter the second number: ');

% If both are odd, add them together
if mod(firstNumber, 2) ~= 0 && mod(secondNumber, 2) ~= 0
    
    result = firstNumber + secondNumber;

% If both are even, subtract the smaller from the larger
elseif mod(firstNumber, 2) == 0 && mod(secondNumber, 2) == 0
    
    result = abs(firstNumber - secondNumber);

% If one is odd and the other is even, multiply them
else
    
    result = firstNumber * secondNumber;

end

% Display the result
fprintf('The result is: %g\n', result);