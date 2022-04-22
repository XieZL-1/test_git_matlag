function INS_Status_num = INS_Status_Cell2num(INS_Status_Fusion_KVH1750)
INS_Status_num = zeros(size(INS_Status_Fusion_KVH1750));
[temp_0,~] = ismember(INS_Status_Fusion_KVH1750,'INS_INACTIVE');
[temp_1,~] = ismember(INS_Status_Fusion_KVH1750,'INS_ALIGNING');
[temp_2,~] = ismember(INS_Status_Fusion_KVH1750,'INS_HIGH_VARIANCE');
[temp_3,~] = ismember(INS_Status_Fusion_KVH1750,'INS_SOLUTION_GOOD');
[temp_6,~] = ismember(INS_Status_Fusion_KVH1750,'INS_SOLUTION_FREE');
[temp_7,~] = ismember(INS_Status_Fusion_KVH1750,'INS_ALIGNMENT_COMPLETE');
Binary_0 = find(temp_0 == 1);
Binary_1 = find(temp_1 == 1);
Binary_2 = find(temp_2 == 1);
Binary_3 = find(temp_3 == 1);
Binary_6 = find(temp_6 == 1);
Binary_7 = find(temp_7 == 1);

INS_Status_num(Binary_0) = 0;
INS_Status_num(Binary_1) = 1;
INS_Status_num(Binary_2) = 2;
INS_Status_num(Binary_3) = 3;
INS_Status_num(Binary_6) = 6;
INS_Status_num(Binary_7) = 7;
end