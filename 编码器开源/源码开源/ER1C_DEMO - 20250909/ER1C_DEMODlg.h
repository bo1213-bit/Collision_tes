// ER1C_DEMODlg.h: 头文件
//

#pragma once


// CER1CDEMODlg 对话框
class CER1CDEMODlg : public CDialogEx
{
// 构造
public:
	CER1CDEMODlg(CWnd* pParent = nullptr);	// 标准构造函数

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ER1C_DEMO_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnBnClickedButtonConnect();


public:
	modbus_t* m_ctx;	//modbus句柄
	CComboBox m_ComboCom;
	CComboBox m_ComboBaudrate;
	CComboBox m_ComboParity;
	bool m_bConnected;	// 是否已连接
	CWinThread *m_ModbusThread;	// Modbus线程
	afx_msg void OnBnClickedButtonDisconnect();

};
