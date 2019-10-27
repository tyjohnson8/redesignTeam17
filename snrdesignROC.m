% Gathering Results

% Conditionals
EQ_thresh = 69;  % if >=, then pass
hearing_thresh = 2;

% Gather "All Data"
% Calculate hearing pass and fails
% Organize hearing and balance Cells with truth and rating
% Convert to csvs
% Make ROCs for both hearing and balance
% Save figures


% How to make a csv file
% You can first create a cell array which preserves the same structure as you want to get displayed in the csv file.
% Next, to import it to a .csv file or .xlsx file, use the xlswrite command with the filename,cellarray, sheet and desired range.

[baseName, folder] = uigetfile('.csv');
AllData = csvread(fullfile(folder, baseName),1);

% Delete initial max angles (row 3)

% Condition = Row 1, Column 3/14/25
% Hearing Errors = Row 1, Column 4/15/26
% Balance Scores = Row (find last column elem = 0 OR last row), Column 5,6,7/16,17,18/27,28,29

% Getting conditions column
truth = AllData(1,3:11:end)';
numSub = length(truth);

% Getting Hearing Columns
hearingUnmod = AllData(1, 4:11:end)';
hearingMod = hearingUnmod;
hearingMod(hearingUnmod > hearing_thresh) = 1;
hearingMod(hearingUnmod <= hearing_thresh) = 0;

% After row 3, within columns 5-10, find the last row with only 3
% nonzero values

% Getting Balance Columns
balanceUnmod = zeros(length(hearingUnmod),3);
len = size(AllData,2);

% Assemble matrix
add = 0;
for L = 1:numSub
    
    myCol(1, 1+add*6:1+add*6+5) = 5+add*11:10+add*11;
    
    add = add + 1;
end


findEQRows = AllData(4:end, myCol);

% find the rows that contain the EQ scores
% 1 through 6, 7 : 12

add = 0;
for L = 1:numSub
   
   testCol = findEQRows(:, 1+add*6:1+add*6+5);
   
   for J = 1:size(testCol,1)
       
       if(size(find(testCol(J,:)),2) == 3)
           r(L) = J;
       end
       
   end
   
   add = add + 1;
end

r = r' + 3;


CM = [ [5:11:len]' [6:11:len]' [7:11:len]' ];

for i = 1:6
    
    balanceUnmod(i,:) = AllData(r(i), CM(i,:));

end

balModX = balanceUnmod(:,1);
balModY = balanceUnmod(:,2);
balModZ = balanceUnmod(:,3);

balModX(balModX < EQ_thresh) = 1;
balModX(balModX >= EQ_thresh) = 0;

balModY(balModY < EQ_thresh) = 1;
balModY(balModY >= EQ_thresh) = 0;

balModZ(balModZ < EQ_thresh) = 1;
balModZ(balModZ >= EQ_thresh) = 0;


test(:,:,1) = [truth hearingMod];
test(:,:,2) = [truth balModX];
test(:,:,3) = [truth balModY];
test(:,:,4) = [truth balModZ];

for i = 1:4
    
   [h, AUC] = myROC( test(:,:,i) );
   
   figure(h);
   fprintf('Figure %i AUC is %f\n', i, AUC);
    
end



