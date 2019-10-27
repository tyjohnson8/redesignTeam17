
%% Function to calc ROC Curve

function [h, AUC] = myROC(data)
% Create ordered list of input "rating" column and cycle through each
% value. Not duplicate values
ratings = data(:,2);
possible_values(1,1) = 0;   % Include the (1,1) point as a threshold value
possible_values(2:length(unique(ratings))+1,1) = unique(ratings);
L = length(possible_values);

% Initialize variables
matrix.true_positive = cell(1,L);
matrix.false_positive = cell(1,L);
matrix.true_negative = cell(1,L);
matrix.false_negative = cell(1,L);

%% Compares rating to truth value and decides whether it is true/false positive/negative, depending on the threshold value

for i=1:L
        count1 = 0;   % I have separate counts for each kind of value
        count2 = 0;
        count3 = 0;
        count4 = 0;
    for q=1:length(data)
       
        if(data(q,1)==1 && data(q,2)>possible_values(i,1)) 
            count1 = count1 + 1;  % true positive
        elseif(data(q,1)==0 && data(q,2)>possible_values(i,1))
            count2 = count2 + 1;  % false positive
        end
        
        if(data(q,1)==0 && data(q,2)<=possible_values(i,1))
            count3 = count3 + 1;  % true negative
        elseif(data(q,1)==1 && data(q,2)<=possible_values(i,1))
            count4 = count4 + 1;  % false negative
        end
        
        matrix.true_positive{1,i} = count1;   % true positive
        matrix.false_positive{1,i} = count2; % false positive
        matrix.true_negative{1,i} = count3; % true negative
        matrix.false_negative{1,i} = count4; % false negative
    end
end

%% Plotting ROC and getting Area
% TPF vs. FPF
% Sensitivity vs. 1-Specificity
% Endpoints always at (0,0) and (1,1)

% Preallocation
TPF = zeros(1,L);  
FPF = zeros(1,L);

% Calculation
for i=1:L
    FPF(1,i) = 1 - (matrix.true_negative{1,i}/(matrix.true_negative{1,i} + matrix.false_positive{1,i}));
    TPF(1,i) = (matrix.true_positive{1,i}/(matrix.true_positive{1,i} + matrix.false_negative{1,i}));
end

h = figure('visible', 'off');
plot(FPF,TPF,'b--o')
hold on; fill([0,1,FPF],[0,0,TPF],[0.7 1 0.7], 'FaceAlpha', 0.5, 'EdgeColor', 'none')
xlabel('FPF (1 - specificity)')
ylabel('TPF (sensitivity)')
title('Receiver Operating Characteristic (ROC) Curve')

% Calculating Area
AUC = -1*trapz(FPF,TPF,2); %#ok<*NASGU> % multiply by negative 1 because it calculates area backwards normally


end