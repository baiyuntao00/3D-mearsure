
// 3DmearsureDlg.h: 头文件
//

#pragma once
#include "mcamera.h"
#include <iostream>
#include "ObjectInfo.h"
// CMy3DmearsureDlg 对话框
class CMy3DmearsureDlg : public CDialogEx
{
// 构造
public:
	CMy3DmearsureDlg(CWnd* pParent = nullptr);	// 标准构造函数

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_MY3DMEARSURE_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;
	mcamera cam;
	bool is_start = false;
	CString str_time;
	CString work_txt;
	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButstart();
	afx_msg void OnBnClickedButopen();
	afx_msg void OnBnClickedOk2();
};
