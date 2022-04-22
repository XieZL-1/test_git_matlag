function Pos_Type_num = Pos_Type_Cell2num(Pos_Type_Fusion_KVH1750)
Pos_Type_num = zeros(size(Pos_Type_Fusion_KVH1750));
[temp_0,~] = ismember(Pos_Type_Fusion_KVH1750,'NONE');
[temp_1,~] = ismember(Pos_Type_Fusion_KVH1750,'FIXEDPOS');
[temp_2,~] = ismember(Pos_Type_Fusion_KVH1750,'FIXEDHEIGHT');
[temp_8,~] = ismember(Pos_Type_Fusion_KVH1750,'DOPPLER_VELOCITY');
[temp_16,~] = ismember(Pos_Type_Fusion_KVH1750,'SINGLE');
[temp_17,~] = ismember(Pos_Type_Fusion_KVH1750,'PSRDIFF');
[temp_18,~] = ismember(Pos_Type_Fusion_KVH1750,'WAAS');
[temp_19,~] = ismember(Pos_Type_Fusion_KVH1750,'PROPAGATED');
[temp_21,~] = ismember(Pos_Type_Fusion_KVH1750,'L1_FLOAT');
[temp_34,~] = ismember(Pos_Type_Fusion_KVH1750,'NARROW_FLOAT');
[temp_48,~] = ismember(Pos_Type_Fusion_KVH1750,'L1_INT');
[temp_49,~] = ismember(Pos_Type_Fusion_KVH1750,'WIDE_INT');
[temp_50,~] = ismember(Pos_Type_Fusion_KVH1750,'NARROW_INT');
[temp_51,~] = ismember(Pos_Type_Fusion_KVH1750,'RTK_DIRECT_INS');
[temp_52,~] = ismember(Pos_Type_Fusion_KVH1750,'INS_SBAS');
[temp_53,~] = ismember(Pos_Type_Fusion_KVH1750,'INS_PSRSP');
[temp_54,~] = ismember(Pos_Type_Fusion_KVH1750,'INS_PSRDIFF');
[temp_55,~] = ismember(Pos_Type_Fusion_KVH1750,'INS_RTKFLOAT');
[temp_56,~] = ismember(Pos_Type_Fusion_KVH1750,'INS_RTKFIXED');
[temp_68,~] = ismember(Pos_Type_Fusion_KVH1750,'PPP_CONVERGING');
[temp_69,~] = ismember(Pos_Type_Fusion_KVH1750,'PPP');
[temp_70,~] = ismember(Pos_Type_Fusion_KVH1750,'OPERATIONAL');
[temp_71,~] = ismember(Pos_Type_Fusion_KVH1750,'WARNING');
[temp_72,~] = ismember(Pos_Type_Fusion_KVH1750,'OUT_OF_BOUNDS');
[temp_73,~] = ismember(Pos_Type_Fusion_KVH1750,'INS_PPP_CONVERGING');
[temp_74,~] = ismember(Pos_Type_Fusion_KVH1750,'INS_PPP');
[temp_77,~] = ismember(Pos_Type_Fusion_KVH1750,'PPP_BASIC_CONVERGING');
[temp_78,~] = ismember(Pos_Type_Fusion_KVH1750,'PPP_BASIC');
[temp_79,~] = ismember(Pos_Type_Fusion_KVH1750,'INS_PPP_BASIC_CONVERGING');
[temp_80,~] = ismember(Pos_Type_Fusion_KVH1750,'INS_PPP_BASIC');

Binary_0 = find(temp_0 == 1);
Binary_1 = find(temp_1 == 1);
Binary_2 = find(temp_2 == 1);
Binary_8 = find(temp_8 == 1);
Binary_16 = find(temp_16 == 1);
Binary_17 = find(temp_17 == 1);
Binary_18 = find(temp_18 == 1);
Binary_19 = find(temp_19 == 1);
Binary_21 = find(temp_21 == 1);
Binary_34 = find(temp_34 == 1);
Binary_48 = find(temp_48 == 1);
Binary_49 = find(temp_49 == 1);
Binary_50 = find(temp_50 == 1);
Binary_51 = find(temp_51 == 1);
Binary_52 = find(temp_52 == 1);
Binary_53 = find(temp_53 == 1);
Binary_54 = find(temp_54 == 1);
Binary_55 = find(temp_55 == 1);
Binary_56 = find(temp_56 == 1);
Binary_68 = find(temp_68 == 1);
Binary_69 = find(temp_69 == 1);
Binary_70 = find(temp_70 == 1);
Binary_71 = find(temp_71 == 1);
Binary_72 = find(temp_72 == 1);
Binary_73 = find(temp_73 == 1);
Binary_74 = find(temp_74 == 1);
Binary_77 = find(temp_77 == 1);
Binary_78 = find(temp_78 == 1);
Binary_79 = find(temp_79 == 1);
Binary_80 = find(temp_80 == 1);

Pos_Type_num(Binary_0) = 0;
Pos_Type_num(Binary_1) = 1;
Pos_Type_num(Binary_2) = 2;
Pos_Type_num(Binary_8) = 8;
Pos_Type_num(Binary_16) = 16;
Pos_Type_num(Binary_17) = 17;
Pos_Type_num(Binary_18) = 18;
Pos_Type_num(Binary_19) = 19;
Pos_Type_num(Binary_21) = 21;
Pos_Type_num(Binary_34) = 34;
Pos_Type_num(Binary_48) = 48;
Pos_Type_num(Binary_49) = 49;
Pos_Type_num(Binary_50) = 50;
Pos_Type_num(Binary_51) = 51;
Pos_Type_num(Binary_52) = 52;
Pos_Type_num(Binary_53) = 53;
Pos_Type_num(Binary_54) = 54;
Pos_Type_num(Binary_55) = 55;
Pos_Type_num(Binary_56) = 56;
Pos_Type_num(Binary_68) = 68;
Pos_Type_num(Binary_69) = 69;
Pos_Type_num(Binary_70) = 70;
Pos_Type_num(Binary_71) = 71;
Pos_Type_num(Binary_72) = 52;
Pos_Type_num(Binary_73) = 73;
Pos_Type_num(Binary_74) = 74;
Pos_Type_num(Binary_77) = 77;
Pos_Type_num(Binary_78) = 78;
Pos_Type_num(Binary_79) = 79;
Pos_Type_num(Binary_80) = 80;

end