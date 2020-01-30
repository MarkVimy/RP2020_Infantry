#include "init.h"
#include "scheduler.h"
 
/**
 *	-2019/11/17
 *	@note
 *	1. ����ͷ��λ�ù̶���Ҫ�ȶ����ô󵯱�������
 *	2. ��̨���е�������Ϊ��ƫ�Ƶ�ʱ���ܹ���������֤�ܹ��������
 *	3. ���������ݷ�����Ư�ƣ�������һ�»����ذ壨��������������ǳ�ʼ���õ����⣬�ѽ����
 *	4. ע��ش�����ص磬��ң�صȵ�ϸ������
 *	5. �ж�����Ѫ������ܹھ���
 *	6. �鳤�ල��Ա��������������ĶԽ�ʱ��
 *	7. ��λ�Ĳ��Է�������������Զ��һ��ǽ��ʮ�֣���еģʽ����ͷ��һ�»���֮������е�Ƕ��Ƿ�һ��
 *	8. ����������̣� 1����ȷ���������ȶ���
 *					 2����Բ�������������ߵ����⣩
 *					 3�����ٿ���
 *  9. ���롢ʵ�֡�����������Ϊ�����������飬��������
 *	10. ����ʵ�ʵ��ϳ�״̬�������Ի�������������ʵս����һ�� 
 *
 *	# ����ܹھ��� #
 *	# ����Infantry #q
 */
  
/**
 *	-2019/01/08
 *	@note
 *	���ǿ�����֮��ĵ�һ�췵����ֻ�ж������ޱ�ִ�ŵ��ˣ��Ż�Թھ�������η��Ұ�ġ�
 *	���Լ���Ҫ��һ���ٵ�ֻ����ս���ϱ�¶���Լ���������
 *
 *	@task
 *	1. �����Ļ������ָܻ�(���ذ�Ĺ̶�ͭ��װ��ȥ֮���ٲ�����̨)
 *		�������ܣ�
 *		�� ң�أ�����	��
 *		�� ���				��
 *		�� Ħ����			��
 *		�� ����				��
 *		�� ����				
 *		�� ��̨
 *
 *	2. С���ݲ���
 *		
 *	@problem
 *	1. �������̵�ʱ�򣬵��̲��ܹ�����ң�ص����������pid�м��㣬�������ֵ�о���Щ�쳣��
 *	2. ��ͼֽ����������ľ���(7.5m)��
 *	3. ���̲����������ƣ�ת���ˡ�(�Ų�֮�����Ƿֵ�壬�ֱ�����һ��)
 */
 
/**
 *	-2019/01/09
 *	@task
 *	��	1. �����Ļ������ָܻ�
 *		�������ܣ�
 *		�� ����	��
 *		�� ��̨	��
 *			- ����pitch�Ļ�е�Ƕȷ�Χ����ֵ
 *			- ����yaw�Ļ�е�Ƕȷ�Χ����ֵ
 *			- ��еģʽ��̨pid����
 *			- ������ģʽ��̨pid����
 *
 *	��	2. С���ݲ���
 *		
 *	@problem
 *	1. ���̵ĵ����������ײ�����³�ʼ�����߲����Ӵ������򽺣��������Լ�С��������ĳ��֡�
 *	
 */
 
/**
 *	-2019/01/10
 *	@task
 *	��	1. С�����˶��Ż�
 *	��	2. ����С���Ե���(����3pin�ߣ�С���ԵĹ�����(�����Ȳ�����̨����һ·��ֳ�����С������))
 *	��	3. ���Ӿ�����ͨ��Э��Ĵ���
 *	��	4. ���Ե���
 *	��	5. ��������(���Կ�Ч����δ��)
 *	��	6. ���Դ��(���Կ�Ч����δ��)
 *		
 *	@problem
 *	1. ���̵ĵ����������ײ�����³�ʼ�����߲����Ӵ������򽺣��������Լ�С��������ĳ��֡�
 *		-2020/01/10	���ִ�����2
 *
 *	2. �о����̵Ļ�е�Ƕ�Ӧ�����ۼӽǶ�ֵ
 *			�������Լ��ı߽紦���ϳ������⡣�޸Ĺ���֮��С���ݾ��ܹ������н���(����)��ͬʱ������Ҳ���Զ�ѡ��ͽ�����
 *
 *	3. ������ֱ��
 *			δ��
 *	4. 
 */

/**
 *	-2019/01/11
 *	@task
 *	��	1. С�����˶��Ż�
 *	��	2. ���Ӿ�����ͨ�������룬�Ӿ������������л�ִ�к���
 *	��	3. �Ӿ��Ǳ�д����������
 *	��	4. ����С��������Ļ���Ч��
 *	��	5. ����Ч�����֣��������Ӽ��ٶ���������ٶ�
 *		
 *	@problem
 *	1. ���̵ĵ����������ײ�����³�ʼ�����߲����Ӵ������򽺣��������Լ�С��������ĳ��֡�
 *		-2020/01/10	���ִ�����2
 *
 *	2. ң�����ݳ������³�����������
 *		-2020/01/11 ���ִ�����1
 *	
 *	3. С������̨ת����ʱ����������������������
 */
 
/**
 *	-2019/01/12
 *	@task
 *	��	1. С�����˶��Ż�
 *	��	2. ����С��������Ļ���Ч��
 *	��	3. ����Ч�����֣��������Ӽ��ٶ���������ٶ�
 *	��	4. С������̨ת����ʱ����������������������
 *		
 *	@problem
 *	1. ���̵ĵ����������ײ�����³�ʼ�����߲����Ӵ������򽺣��������Լ�С��������ĳ��֡�
 *		-2020/01/10	���ִ�����2
 *
 *	2. ң�����ݳ������³�����������
 *		-2020/01/11 ���ִ�����1
 *	
 *	3. С������̨ת����ʱ����������������������
 */
 
/**
 *	-2019/01/16
 *	@task
 *	��	1. С�����˶��Ż�
 *	��	2. ����С��������Ļ���Ч��
 *	��	3. ����Ч�����֣��������Ӽ��ٶ���������ٶ�
 *		
 *	@problem
 *	1. ���̵ĵ����������ײ�����³�ʼ�����߲����Ӵ������򽺣��������Լ�С��������ĳ��֡�
 *		-2020/01/10	���ִ�����2
 *
 *	2. ң�����ݳ������³�����������
 *		-2020/01/11 ���ִ�����1
 *	
 *	3. С������̨ת����ʱ����������������������
 *
 *	4. ����ģʽ�µ��̱���
 *		-2020/01/17 ���ִ�����2
 *
 *	@note
 *	1. ƽ�Ƶ�ʱ�򣬿����Ȼ�������һ��ʱ���ٽ��ٶ�����ȥ��
 *
 */
 
/**
 *	-2019/01/17
 *	@task
 *	��	1. С�����˶��Ż�
 *	��	2. ����С��������Ļ���Ч��
 *	��	3. ����Ч�����֣��������Ӽ��ٶ���������ٶ�
 *		
 *	@problem
 *	1. ���̵ĵ����������ײ�����³�ʼ�����߲����Ӵ������򽺣��������Լ�С��������ĳ��֡�
 *		-2020/01/10	���ִ�����2
 *		-2020/01/17	���ִ�����1
 *
 *	2. ң�����ݳ������³�����������
 *		-2020/01/11 ���ִ�����1
 *	
 *	3. ����ģʽ�µ��̱���
 *		-2020/01/16 ���ִ�����2
 *		-2020/01/17 ���ִ�����1
 *
 *	4. ң��ģʽ�µ��̱���
 *		-2020/01/17 ���ִ�����1
 *
 *	5. ��Ħ���ֲ�ת
 *
 *	@note
 *	1. ƽ�Ƶ�ʱ�򣬿����Ȼ�������һ��ʱ���ٽ��ٶ�����ȥ��
 */
 
 /**
  *	@problem
	*	���̱���ԭ�������
	*	- ����CAN�߶�����������PIDը��
	*	- �������߼����������������
	*	- ң�����ݳ���/ʧ����û��ʧ��/��������
	*/
	
/**
 *	@note
 *	�������ҵ��Է�����
 *	��ַ����� - ��ҡ�˲������Ͻǣ�ʹ�ý���������������ת���۲쳵��ƫת��������������
 *							�����˲������Ͻǣ�ʹ�ý���������������ת���۲쳵��ƫת��������������
 *
 *	�ص�ש��ֱ�߷� - ��������׼��ש�ķ죬ҡ����ֱ�ߣ����õ����ֱ�����жϳ���ƫת��������������
 *
 *	ƽ�Ʒ� - 
 */
 
/**
 *	������ʱ��һ��Ҫ�����һ����ӳ��ߣ���Ȼ���ǵ��ߣ�-_-\\
 *
 */
 
 int main(void)
{

	All_Init();
	while(1) {
	}
}