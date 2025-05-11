% 示例输入位姿
x0 = [0; 0; 0; 0; 0; 0];



[Jinv, sv, condJ] = analyzeJacobianNonDim(x0);

fprintf('伪逆雅可比条件数：%.2f\n', condJ);
disp('伪逆雅可比的奇异值：');
disp(sv);
