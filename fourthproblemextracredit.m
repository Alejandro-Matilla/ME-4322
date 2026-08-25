% Create an 'empty' array to store the 10 random numbers
randomNumbers = zeros(1, 10);

% Generate 10 random numbers using a for loop
for i = 1:10
    randomNumbers(i) = rand;
end

% Create x-axis values from 1 to 10
x = 1:10;

% Create the plot
plot(x, randomNumbers);

%Setting the values of x to show on the graph from 1 - 10
% so the graph starts at x = 1 and ends at x = 10
xlim([1 10]);

% Label the axes
xlabel('Number');
ylabel('Random Number');
