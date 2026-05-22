// ER1C_DEMODlg.cpp: 实现文件
//

#include "pch.h"
#include "framework.h"
#include "ER1C_DEMO.h"
#include "ER1C_DEMODlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// 采集器modbus保持寄存器定义
#define RW_REG_SLAVE_ADDR		0	//采集器地址：占1寄存器
#define RW_REG_BAUD_RATE		1	//波特率：占2寄存器
#define RW_REG_PARITY				3	//校验位：占1寄存器
//4~19空

#define RW_REG_ENC_PROTO		20	//编码器协议，BISS-C, ABZ, 多摩川
#define RW_REG_MT_BITS			21	//多圈数据位数
#define RW_REG_ST_BITS			22	//单圈数据位数

#define RW_REG_ENC_TYPE			23	//1REG, 编码器类型：0=直线型，1=旋转型
#define RW_REG_LINEAR_RES		24	//2REG, 无符号整型，直线型编码器分辨率：nm
#define RW_REG_ROTARY_RES		26	//2REG, 无符号整型，旋转型编码器分辨率：counts/rev
#define RW_REG_COUNT_DIR 		28	//1REG, 增量编码器计数方向，0=原始，1=取反
#define RW_REG_COUNT_MODE		29	//1REG, 增量编码器计数模式，0=正交，1=脉冲+方向，2=正反脉冲；
#define RW_REG_FILTER_LEVEL	30	//1REG, 增量编码器滤波等级；
#define RW_REG_ABS_FREQ			31	//1REG, 绝对值编码器通讯频率
#define RW_REG_ABS_OFFSET		32	//4REG, 绝对值编码器偏移量；
#define RW_REG_AUTO_ZERO		36	//1REG, 增量式编码器自动清零，0=不清零，1=上电过Z清零；
#define RW_REG_HOMING_DIR		37	//1REG, 增量式编码器归零方向，0=负方向过Z清零，1=正方向过Z清零
//38~99空

//从100开始不可保存
#define RW_REG_ENC_VALUE		100	//编码器数据（多圈+单圈），占4个寄存器
#define RW_REG_ERROR_BIT		104	//BISS-C的错误位，占1寄存器
#define RW_REG_WARN_BIT			105	//BISS-C的警告位，占1寄存器
#define RW_REG_RECV_CRC			106	//BISS-C接收到的CRC值，占1寄存器
#define RW_REG_CRC_RESULT		107	//1REG,CRC校验结果
#define RW_REG_ENC_CORRECT	108	//4REG, 绝对值编码器数据校正值（减去偏移量）
#define RW_REG_ENC_CAPTURED	112	//4REG, 增量编码器捕获值

#define RW_REG_SINGLE_TURN	116	//2REG, 多摩川绝度值编码器单圈值
#define RW_REG_MULTI_TURN		118	//2reg, 多摩川绝度值编码器多圈值
#define RW_REG_TAMAG_SF			120	//1REG, 多摩川编码器状态域
#define RW_REG_TAMAG_ALMC		121	//1REG, 多摩川编码器的报警码
#define RW_REG_TAMAG_ENID		122	//1REG, 多摩川编码器的ID，一般默认为单圈位数

#define RW_REG_ZERO_SW			123		//1reg, 增量式编码器清零功能开关，1=开启过经过Z清零，0=关闭
#define RW_REG_ZERO_STATUS	124		//1reg, 增量式编码器清零完成状态，0=初始状态，1=清零中， 2=清零完成


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CER1CDEMODlg 对话框



CER1CDEMODlg::CER1CDEMODlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_ER1C_DEMO_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CER1CDEMODlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO_COM, m_ComboCom);
	DDX_Control(pDX, IDC_COMBO_BAUDRATE, m_ComboBaudrate);
	DDX_Control(pDX, IDC_COMBO_PARITY, m_ComboParity);
}

BEGIN_MESSAGE_MAP(CER1CDEMODlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_CREATE()
	ON_BN_CLICKED(IDC_BUTTON_CONNECT, &CER1CDEMODlg::OnBnClickedButtonConnect)
	ON_BN_CLICKED(IDC_BUTTON_DISCONNECT, &CER1CDEMODlg::OnBnClickedButtonDisconnect)
END_MESSAGE_MAP()


// CER1CDEMODlg 消息处理程序

BOOL CER1CDEMODlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码
	m_ComboBaudrate.SetCurSel(4);
	m_ComboParity.SetCurSel(2);

	SetDlgItemInt(IDC_EDIT_SLAVE_ADDR, 1); // 设置默认从站地址为1
	m_ModbusThread = NULL; // 初始化Modbus线程为NULL

	//以下段是枚举串口，加入ComboBox串口列表中
	// 获取ComboBox控件
	CComboBox* pCombo = (CComboBox*)GetDlgItem(IDC_COMBO_COM);
	if (!pCombo)
	{
		AfxMessageBox(_T("未找到串口选择控件"));
		return FALSE;
	}

	// 清空ComboBox
	pCombo->ResetContent();

	// 枚举可用串口
	CString strPort;
	for (int i = 1; i <= 255; i++)
	{
		strPort.Format(_T("COM%d"), i);

		// 尝试打开串口
		HANDLE hPort = CreateFile(
			strPort,
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			0,
			NULL
		);

		if (hPort != INVALID_HANDLE_VALUE)
		{
			// 如果成功打开，说明串口存在
			pCombo->AddString(strPort);
			CloseHandle(hPort);  // 立即关闭句柄
		}
		else
		{
			// 检查错误是否为拒绝访问（说明串口存在但被占用）
			if (GetLastError() == ERROR_ACCESS_DENIED)
			{
				pCombo->AddString(strPort);
			}
		}
	}

	// 设置默认选择（可选）
	if (pCombo->GetCount() > 0)
	{
		pCombo->SetCurSel(pCombo->GetCount()-1);
	}

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CER1CDEMODlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CER1CDEMODlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CER1CDEMODlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


int CER1CDEMODlg::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CDialogEx::OnCreate(lpCreateStruct) == -1)
		return -1;

	// TODO:  在此添加您专用的创建代码

// 在您的对话框初始化函数（如OnInitDialog）中添加以下代码


	return 0;
}


UINT ModbusThread(LPVOID pParam)
{
    CER1CDEMODlg* pDlg = (CER1CDEMODlg * )pParam;

	// 读取编码器协议的保持寄存器
	uint16_t enc_proto = 0;
	modbus_read_registers(pDlg->m_ctx, RW_REG_ENC_PROTO, 1, &enc_proto);

	//读取编码器多圈和单圈数据位数
	uint16_t mt_bits = 0;
	modbus_read_registers(pDlg->m_ctx, RW_REG_MT_BITS, 1, &mt_bits);

    // 示例：循环读取保持寄存器
    while (pDlg->m_ctx)
    {
        uint16_t tab_reg[4] = { 0 };
        int rc = modbus_read_registers(pDlg->m_ctx, RW_REG_ENC_VALUE, 4, tab_reg);
        if (rc != -1)
        {
			// 显示编码器值
			if (enc_proto == 1)	//ABZ编码器，编码器值是一个有符号数
			{
				int64_t enc_value = *((int64_t*)tab_reg);
				pDlg->SetDlgItemInt(IDC_EDIT_ENCODER_DATA, enc_value);
			}
			else
			{
				// 绝对值协议的编码器，
				if(mt_bits == 0) //单圈绝对值编码器，编码器值表示为有符号数，以便当单圈溢出时，扩展到多圈
				{
					int64_t enc_value = *((int64_t*)tab_reg);
					pDlg->SetDlgItemInt(IDC_EDIT_ENCODER_DATA, enc_value);
				}
				else //多圈绝对值编码器，编码器值表示为无符号数
				{
					uint64_t enc_value = *((uint64_t*)tab_reg);
					pDlg->SetDlgItemInt(IDC_EDIT_ENCODER_DATA, enc_value, FALSE);
				}
			}	
		}
		else
		{
	
        }
        Sleep(100); // 100ms
    }

	pDlg->m_ModbusThread = NULL; // 线程结束时将指针设置为NULL
    return 0;
}

void CER1CDEMODlg::OnBnClickedButtonConnect()
{
    // TODO: 在此添加控件通知处理程序代码

    if (m_ctx)
        modbus_close(m_ctx);

    CString strCom;	//com口字符串
    m_ComboCom.GetLBText(m_ComboCom.GetCurSel(), strCom);

    int baud = GetDlgItemInt(IDC_COMBO_BAUDRATE);	// 波特率

    char parity;	// 奇偶校验
    switch (m_ComboParity.GetCurSel())
    {
    case 0:
        parity = 'O'; // 奇校验
        break;
    case 1:
        parity = 'E'; // 偶校验
        break;
    case 2:
        parity = 'N'; // 无奇偶校验
        break;
    default:
        parity = 'N';// 无奇偶校验
        break;
    }

    USES_CONVERSION;
    m_ctx = modbus_new_rtu(T2A(strCom), baud, parity, 8, 1);

    int slave_addr = GetDlgItemInt(IDC_EDIT_SLAVE_ADDR); // 获取从站地址
    modbus_set_slave(m_ctx, slave_addr);

    if (modbus_connect(m_ctx) == -1)
    {
		modbus_free(m_ctx);
		m_ctx = NULL;
		MessageBox(_T("连接失败！"));
    }
    else
    {
		//读一个寄存器测试连接
		uint16_t test_reg = 0;
		if (modbus_read_registers(m_ctx, RW_REG_SLAVE_ADDR, 1, &test_reg) == -1)
		{
			modbus_free(m_ctx);
			m_ctx = NULL;
			MessageBox(_T("连接失败！"));
		}
		else
		{
			MessageBox(_T("连接成功！"));
			// 在这里可以添加连接成功后的其他操作
			//reate a thread to handle Modbus communication
			if(m_ModbusThread == NULL)
			{
				m_ModbusThread = AfxBeginThread(ModbusThread, this);
			}

		}
       
    }
}

void CER1CDEMODlg::OnBnClickedButtonDisconnect()
{
	// TODO: 在此添加控件通知处理程序代码
	modbus_close(m_ctx);
	m_ctx = NULL;
	if (m_ModbusThread)
	{
		// 等待线程结束
		WaitForSingleObject(m_ModbusThread->m_hThread, INFINITE);
		m_ModbusThread = NULL;
	}
}

