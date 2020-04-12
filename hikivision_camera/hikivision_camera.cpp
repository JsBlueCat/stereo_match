#include "hikivision_camera.h"
/*****
˫Ŀ�����ȡ ͼƬ
***/
void  StereoCamera::GrabImageDoubleCamera() {
	int nRet;
	// get one frame from camera with timeout=1000ms
	nRet = MV_CC_GetOneFrameTimeout(this->handle_left, this->left_pData, this->g_nPayloadSize, &this->stImageInfo_left, 1000);
	if (nRet == MV_OK)
	{
		printf("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n",
			this->stImageInfo_left.nWidth, this->stImageInfo_left.nHeight, this->stImageInfo_left.nFrameNum);
	}
	else
	{
		printf("No data[0x%x]\n", nRet);
		free(this->left_pData);
		this->left_pData = NULL;
		cerr << "system fail";
	}
	nRet = MV_CC_GetOneFrameTimeout(this->handle_right, this->right_pData, this->g_nPayloadSize, &this->stImageInfo_right, 1000);
	if (nRet == MV_OK)
	{
		printf("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n",
			this->stImageInfo_right.nWidth, this->stImageInfo_right.nHeight, this->stImageInfo_right.nFrameNum);
	}
	else
	{
		printf("No data[0x%x]\n", nRet);
		free(this->right_pData);
		this->right_pData = NULL;
		cerr << "system fail";
	}
	bool finish;
	finish = Convert2Mat(&this->stImageInfo_left, this->left_pData, this->left_img);
	if (!finish) {
		cerr << "convert error , system break" << endl;
	}
	finish = Convert2Mat(&this->stImageInfo_right, this->right_pData, this->right_img);
	if (!finish) {
		cerr << "convert error , system break" << endl;
	}
}


/**
��˫Ŀ�����
***/
void  StereoCamera::OpenDoubleCamera(const unsigned int num) {
	cout << "opening stereo camerea" << endl;
	int nRet;
	nRet = MV_CC_OpenDevice(this->handle_left);
	if (MV_OK != nRet)
	{
		cerr << "Open Left Device fail! nRet [0x" << nRet << "]" << endl;
	}

	nRet = MV_CC_OpenDevice(this->handle_right);
	if (MV_OK != nRet)
	{
		cerr << "Open Right Device fail! nRet [0x" << nRet << "]" << endl;
	}
	// ��� ����������ݰ�
	// Detection network optimal package size(It only works for the GigE camera)
	if (stDeviceList.pDeviceInfo[this->left_idx]->nTLayerType == MV_GIGE_DEVICE)
	{
		int nPacketSize = MV_CC_GetOptimalPacketSize(this->handle_left);
		if (nPacketSize > 0)
		{
			nRet = MV_CC_SetIntValue(this->handle_left, "GevSCPSPacketSize", nPacketSize);
			if (nRet != MV_OK)
			{
				printf("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
			}
		}
		else
		{
			printf("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
		}
	}
	// ��� ����������ݰ�
	if (stDeviceList.pDeviceInfo[this->right_idx]->nTLayerType == MV_GIGE_DEVICE)
	{
		int nPacketSize = MV_CC_GetOptimalPacketSize(this->handle_right);
		if (nPacketSize > 0)
		{
			nRet = MV_CC_SetIntValue(this->handle_right, "GevSCPSPacketSize", nPacketSize);
			if (nRet != MV_OK)
			{
				printf("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
			}
		}
		else
		{
			printf("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
		}
	}

	// Set trigger mode as off
	nRet = MV_CC_SetEnumValue(this->handle_left, "TriggerMode", MV_TRIGGER_MODE_OFF);
	if (MV_OK != nRet)
	{
		printf("Cam[%d]: MV_CC_SetTriggerMode fail! nRet [%x]\n", this->left_idx, nRet);
	}
	nRet = MV_CC_SetEnumValue(this->handle_right, "TriggerMode", MV_TRIGGER_MODE_OFF);
	if (MV_OK != nRet)
	{
		printf("Cam[%d]: MV_CC_SetTriggerMode fail! nRet [%x]\n", this->right_idx, nRet);
	}

	nRet = MV_CC_SetGainMode(this->handle_left, num);
	if (MV_OK != nRet)
	{
		printf("���������������ʧ��");
	}
	nRet = MV_CC_SetGainMode(this->handle_right, num);
	if (MV_OK != nRet)
	{
		printf("���������������ʧ��");
	}

	// Get payload size
	// ������� Ĭ����Ϣ��һ�µ�
	MVCC_INTVALUE stParam;
	memset(&stParam, 0, sizeof(MVCC_INTVALUE));
	nRet = MV_CC_GetIntValue(this->handle_left, "PayloadSize", &stParam);
	if (MV_OK != nRet)
	{
		printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
	}
	this->g_nPayloadSize = stParam.nCurValue;

	// Start grab image
	nRet = MV_CC_StartGrabbing(this->handle_left);
	if (MV_OK != nRet)
	{
		printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
		cerr << "systeam break" << endl;
	}
	nRet = MV_CC_StartGrabbing(this->handle_right);
	if (MV_OK != nRet)
	{
		printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
		cerr << "systeam break" << endl;
	}
	//  ��ʼ�� ͼ����Ϣ �Լ� ����buff �ռ�
	memset(&this->stImageInfo_left, 0, sizeof(MV_FRAME_OUT_INFO_EX));
	memset(&this->stImageInfo_right, 0, sizeof(MV_FRAME_OUT_INFO_EX));
	this->left_pData = (unsigned char *)malloc(sizeof(unsigned char) * (this->g_nPayloadSize));
	this->right_pData = (unsigned char *)malloc(sizeof(unsigned char) * (this->g_nPayloadSize));
	if (this->left_pData == NULL)
	{
		printf("Allocate memory failed.\n");
		cerr << "systeam break" << endl;
	}
	if (this->right_pData == NULL)
	{
		printf("Allocate memory failed.\n");
		cerr << "systeam break" << endl;
	}
}

/**
����豸״̬
***/
int  StereoCamera::PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo) {
	//����豸״̬
	if (NULL == pstMVDevInfo)
	{
		printf("The Pointer of pstMVDevInfo is NULL!\n");
	}
	if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
	{
		int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
		int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
		int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
		int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

		// print current ip and user defined name
		printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
		printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
		return nIp4;
	}
	return -1;
}

/**
��ʼ�����캯��
***/
StereoCamera::StereoCamera() {
	//��ʼ���������
	// ��ʼ�����
	int times = 0;
	memset(&this->stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
	do {
		int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE, &this->stDeviceList);
		if (MV_OK != nRet)
		{
			cout << "Enum Devices fail! nRet [0x" << nRet << "]" << endl;
		}
		else {
			// �ɹ� �����豸 
			if (this->stDeviceList.nDeviceNum > DEVICE_NUMBER) {
				// �������������
				for (unsigned int i = 0; i < this->stDeviceList.nDeviceNum; i++)
				{
					printf("[device %d]:\n", i);
					MV_CC_DEVICE_INFO* pDeviceInfo = this->stDeviceList.pDeviceInfo[i];
					if (NULL == pDeviceInfo)
					{
						break;
					}
					int ret = this->PrintDeviceInfo(pDeviceInfo);
					if (ret > 0) {
						this->camera_ip_map[ret] = i;
					}
				}
				break;
			}
			else { // ����һ̨
				cout << "don't catch two camera! please check camera status" << endl;
			}
		}
		times++;
		if (times > 3600) {
			cerr << "too many times to connect camera! please check camera status" << endl;
			break;
		}
	} while (1);
	
	// �ҵ���2�����  ����ʼ����  map 
	int nRet;
	map<int, int>::iterator it = this->camera_ip_map.find(LEFT_CAMERA_IP4);
	if (it != this->camera_ip_map.end()) {
		//  �ҵ������
		this->left_idx = it->second;
		nRet = MV_CC_CreateHandle(&this->handle_left, stDeviceList.pDeviceInfo[it->second]);
		int count = 0;
		while (MV_OK != nRet)
		{
			cerr << "Create Handle fail! nRet [0x" << nRet << "]"<< endl;
			nRet = MV_CC_CreateHandle(&this->handle_left, stDeviceList.pDeviceInfo[it->second]);
			if (count++ > 100) {
				cerr << "��������100��ʧ��,�������������豸" << endl;
				exit(-1);
			}
		}
	}
	it = this->camera_ip_map.find(RIGHT_CAMERA_IP4);
	if (it != this->camera_ip_map.end()) {
		//  �ҵ������
		this->right_idx = it->second;
		nRet = MV_CC_CreateHandle(&this->handle_right, stDeviceList.pDeviceInfo[it->second]);
		int count = 0;
		while (MV_OK != nRet)
		{
			cerr << "Create Handle fail! nRet [0x" << nRet << "]" << endl;
			nRet = MV_CC_CreateHandle(&this->handle_right, stDeviceList.pDeviceInfo[it->second]);
			if (count++ > 100) {
				cerr << "��������100��ʧ��,�������������豸" << endl;
				exit(-1);
			}
		}
	}
}

/**
��ʼ����������
***/
StereoCamera::~StereoCamera() {
	// ��ն�Ӧmap
	this->camera_ip_map.clear();
	this->left_img.release();
	this->right_img.release();
	// Stop grab image
	// ֹͣץȡͼ��
	int nRet;
	nRet = MV_CC_StopGrabbing(this->handle_left);
	if (MV_OK != nRet)
	{
		cerr << "Stop Grabbing Left Device fail! nRet [0x" << nRet << "]" << endl;
	}
	nRet = MV_CC_StopGrabbing(this->handle_right);
	if (MV_OK != nRet)
	{
		cerr << "Stop Grabbing Right Device fail! nRet [0x" << nRet << "]" << endl;
	}
	//�ر������ 
	nRet = MV_CC_CloseDevice(this->handle_left);
	if (MV_OK != nRet)
	{
		cerr << "Close Left Device fail! nRet [0x" << nRet << "]" << endl;
	}
	//�ر������ 
	nRet = MV_CC_CloseDevice(this->handle_right);
	if (MV_OK != nRet)
	{
		cerr << "Close Right Device fail! nRet [0x" << nRet << "]" << endl;
	}
	// Destroy handle
	// ���� ��������
	nRet = MV_CC_DestroyHandle(this->handle_left);
	if (MV_OK != nRet)
	{
		cerr << "Destroy Left Handle fail! nRet [0x" << nRet << "]" << endl;
	}
	// ���� ��������
	nRet = MV_CC_DestroyHandle(this->handle_right);
	if (MV_OK != nRet)
	{
		cerr << "Destroy Right Handle fail! nRet [0x" << nRet << "]" << endl;
	}
}



int RGB2BGR(unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight) {
	if (NULL == pRgbData)
	{
		return MV_E_PARAMETER;
	}

	for (unsigned int j = 0; j < nHeight; j++)
	{
		for (unsigned int i = 0; i < nWidth; i++)
		{
			unsigned char red = pRgbData[j * (nWidth * 3) + i * 3];
			pRgbData[j * (nWidth * 3) + i * 3] = pRgbData[j * (nWidth * 3) + i * 3 + 2];
			pRgbData[j * (nWidth * 3) + i * 3 + 2] = red;
		}
	}

	return MV_OK;
}

bool Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char * pData, cv::Mat &srcImage) {
	if (pstImageInfo->enPixelType == PixelType_Gvsp_Mono8)
	{
		srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC1, pData);
	}
	else if (pstImageInfo->enPixelType == PixelType_Gvsp_RGB8_Packed)
	{
		RGB2BGR(pData, pstImageInfo->nWidth, pstImageInfo->nHeight);
		srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC3, pData);
	}
	else
	{
		printf("unsupported pixel format\n");
		return false;
	}

	if (NULL == srcImage.data)
	{
		return false;
	}
	return true;
}