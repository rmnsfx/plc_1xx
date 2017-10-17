#include "main.h"



//������ ��� �������� ��������
uint16_t settings[REG_COUNT];

uint16_t default_settings[REG_COUNT] = 
{		
		//����� ICP		
		0, 	//1. ���������� ������ ICP
		0, 	//2. 
0x4100,	//3. ������ ������� �����������������
		0,	//4. 
0x4110,	//5. ������� ������� �����������������
		0,	//6. 
0x4110,	//7. ������ ������� ���������
		0,	//8. 
0x4120,	//9. ������� ������� ���������
		0,	//10. 
		0,	//11. ���� ����������� ������ ICP (1 - ��������, 0 - �����)
0x3A03,	//12. ������� ��������� ������
0x126F,	//13. 
		0,	//14. ---
		0,	//15. 
0x3F80,	//16.	����. �������� 
		0,	//17.	
		0,	//18. ����. �������� 
		0,	//19. 
		1,	//20. ����� ��������� �������
0x4396,	//21. �������� ���������� ���.�������� (�������)
		0,	//22. 
		0,	//23. ���������, ���. ������� ���.�������� (������������ ��������)
		0,	//24. 
		0,	//25. ��������, ���. ������� ���.�������� (������������ ��������)
		0,	//26. 
		0,	//27. �����������, ���. ������� ���.�������� (������������ ��������)
		0,	//28. 
		0,	//29. -��������� �������������
		0,	//30. 
		0,	//31. -������ �������������
		0,	//32. 
		0,	//33. 
		0,	//34. 
		0,	//35. 
		0,	//36. 
				
	
		//����� 4-20
		0,	//37. ��� 
		0,	//38. 
0x4180,	//39. ������ ������� �����������������
		0,	//40. 
0x4190,	//41. ������� ������� �����������������
		0,	//42. 
0x4190,	//43.	������ ������� ���������
		0,	//44. 
0x41A0,	//45. ������� ������� ���������
		0,	//46. 
		0,	//47. ���� ����������� ������ 4-20 (1 - ��������, 0 - �����)
0x4076,	//48. ������� ��������� ������
0x6666,	//49. 
		0,	//50. ---
		0,	//51. 
0x3BE5,	//52.	����. �������� 
0x6042,	//53.	
		0,	//54. ����. ��������
		0,	//55. 
0x4580,	//56. �������� �������
		0,	//57. 
		0,	//58. 
		0,	//59. 
		0,	//60. 
		0,	//61. 
		0,	//62. 
		0,	//63. 
		0,	//64. 
		
		//����� 485 (modbus master)
		1,	//65.	����� �������������
0x4616,	//66.	�������� ������
		0,	//67.	
	1000,	//68. �������
		0,	//69. ����� ������� �������� 
		0,	//70. 
	0x3,	//71. �������
		4,	//72. ���������� ���������
		0,	//73. 
		0,	//74. 
		0,	//75. 
		0,	//76. 
		0,	//77. 
		0,	//78. 
		0,	//79. 
		0,	//80. 
		0,	//81. 
		0,	//82. 
		
		//����
		0,	//83. ��������� �������. ���� 
		0,	//84. ��������� ����. ����
		0,	//85. ����� ������ (0 - ��� ������, 1 - � �������)
		0,	//86. �������� ������� (0 - ICP, 1 - 4-20, 2 - 485)
	100,	//87. �������� �� ������������, ��
		0,	//88.
		0,	//89.
		
		//����� 4-20
		0,	//90. �������� ������� ��� ������ (0 - icp, 1 - 4-20, 2 - 485)
		0,	//91. ����. ��������
		0,	//92. 
		0,	//93. ����. ��������
		0,	//94. 
0x41F0,	//95. �������� ��������
		0,	//96. 
		
		//���������� ����
		0,	//97. ������� ��������� (������ 1 � ������� - ������� ������������ ����)
		0,	//98.
				
		//����� ��������� 
		0,	//99.	���������� ������� ����������� 
		0,	//100. 
		10,	//101. ����� ���������� (modbus slave)
0x47E1,	//102. �������� ������
		0,	//103. 
		0,	//104. �������� CPU
		0,	//105. 
0x3F80,	//106. ������ ��
		0,	//107. 		
		0,	//108. ������ �� ������������������� ������
		0,	//109. ����� �� �������� �� ��������� 
0x3E8,	//110. ����� ��������		
0x41B0,	//111. ������ ������� ������� �����������
		0,	//112. 
0x41D0,	//113. ������� ������� ������� �����������
		0,	//114. 		
		
		//����� 485 (modbus master) �����������
		0,	//115. ����� �������� 1
 1000,	//116. ����. �
		0,	//117. ��������
		0,	//118. ������ ����. ������� 1
		0,	//119. ������� ����. ������� 1
		0,	//120. ������ ����. ������� 1
		0,	//121. ������� ����. ������� 1
		0,	//122. ����� �������� 2
		1,	//123. ����. �
		0,	//124. ��������
		0,	//125. ������ ����. ������� 2
		0,	//126. ������� ����. ������� 2
		0,	//127. ������ ����. ������� 2
		0,	//128. ������� ����. ������� 2
		0,	//129. ����� �������� 3
		1,	//130. ����. �
		0,	//131. ��������
		0,	//132. ������ ����. ������� 3
		0,	//133. ������� ����. ������� 3
		0,	//134. ������ ����. ������� 3		
		0,	//135. ������� ����. ������� 3
		0,	//136. ����� �������� 4
		1,	//137. ����. �
		0,	//138. ��������
		0,	//139. ������ ����. ������� 4
		0,	//140. ������� ����. ������� 4
		0,	//141. ������ ����. ������� 4		
		0,	//142. ������� ����. ������� 4

};

