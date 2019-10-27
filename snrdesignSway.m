% Graphs plot sways

[baseName, folder] = uigetfile('.csv');
AllData = csvread(fullfile(folder, baseName),1);


len = size(AllData,2);
truth = AllData(1,3:11:end)';
numSub = length(truth);


% Find columns
% Assemble matrix
add = 0;
for L = 1:numSub
    
    myCol(1, 1+add*4:1+add*4+3) = 5+add*11:8+add*11;
    
    add = add + 1;
end

% Find end rows
findEQRows = AllData(4:end, myCol);

% find the rows that contain the EQ scores
% 1 through 6, 7 : 12

add = 0;
for L = 1:numSub
   
   testCol = findEQRows(:, 1+add*4:4+add*4);
   
   for J = 1:size(testCol,1)
       
       if(size(find(testCol(J,:)),2) == 3)
           r(L) = J;
       end
       
   end
   
   add = add + 1;
end

r_end = r' + 3;


% Error vals
for L = 0:numSub-1
    
    if(find( (5+L*11:8+L*11) > len ) )
        break;
    end
    
    CMerrors(1, 1+L*4:4+L*4) = 5+L*11:8+L*11;
    
end
AllMeanDrifts = AllData(1, CMerrors);
AllMeanDrifts = reshape(AllMeanDrifts, 4, numSub)';
AllMeanDrifts(:,4) = [];

% Std Vals
AllMeanStds = AllData(2, CMerrors);
AllMeanStds = reshape(AllMeanStds, 4, numSub)';
AllMeanStds(:,4) = [];

% Sway Data
PreAllSway = AllData(3:r_end-2, CMerrors);
% Find columns for AllSway
for L = 0:numSub-1
    
    AllSway(:,:,L+1) = PreAllSway(:,1+L*4:4+L*4);
    
end







