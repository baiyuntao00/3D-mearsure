
// 3DmearsureDlg.cpp: 实现文件
//

#include "pch.h"
#include "framework.h"
#include "3Dmearsure.h"
#include "3DmearsureDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif
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


// CMy3DmearsureDlg 对话框



CMy3DmearsureDlg::CMy3DmearsureDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_MY3DMEARSURE_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CMy3DmearsureDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CMy3DmearsureDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(ID_BUTSTART, &CMy3DmearsureDlg::OnBnClickedButstart)
	ON_BN_CLICKED(ID_BUTOPEN, &CMy3DmearsureDlg::OnBnClickedButopen)
	ON_BN_CLICKED(IDOK2, &CMy3DmearsureDlg::OnBnClickedOk2)
END_MESSAGE_MAP()


// CMy3DmearsureDlg 消息处理程序

BOOL CMy3DmearsureDlg::OnInitDialog()
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
	//工作面板初始化
	str_time = CTime::GetCurrentTime().Format("%X：");
	work_txt += (str_time + _T(" 点击开始！\n"));
//	GetDlgItem(IDC_TEXTWORK)->SetWindowText(work_txt);
	//调整空间大小
	CRect rect_pict_pre;
	GetDlgItem(IDC_CVMAT)->GetWindowRect(&rect_pict_pre);
	GetDlgItem(IDC_CVMAT)->MoveWindow(0, 35, 640, 480, true);
	//MFC显示opencv的mat图像
	namedWindow("opencv", WINDOW_AUTOSIZE);
	HWND hWnd = (HWND)cvGetWindowHandle("opencv");
	HWND hParent = ::GetParent(hWnd);
	::SetParent(hWnd, GetDlgItem(IDC_CVMAT)->m_hWnd);
	::ShowWindow(hParent, SW_SHOW);
	//相机初始化，获取相机的内参，深度比例
	cam = mcamera(480, 640);
	if (!cam.initCamera())
	{
		work_txt += (str_time + _T(" 没有检测到相机!\n"));
		GetDlgItem(IDC_TEXTWORK)->SetWindowText(work_txt);
		is_start = false;
	}
	else {
		size_t size = cam.name.length();
		wchar_t* buffer = new wchar_t[size + 1];
		MultiByteToWideChar(CP_ACP, 0, cam.name.c_str(), size, buffer, size * sizeof(wchar_t));
		buffer[size] = 0;
		//delete buffer;
		GetDlgItem(IDC_CAMNAME)->SetWindowText(buffer);
		delete buffer;
	}
	is_result = false;
	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CMy3DmearsureDlg::OnSysCommand(UINT nID, LPARAM lParam)
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

void CMy3DmearsureDlg::OnPaint()
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
	while (waitKey(100) < 0)
	{
		if (is_result)
		{
			while (waitKey(100) < 0)
				if (is_start)
				{
					is_result = false;
					Sleep(500);
					is_start = false;
					break;
				}
		}
		bool to = cam.getImage();
		if (is_start&&to&&!is_result)//开始计算
		{
			is_start = false;
			cam.getPointCloud();
			ObjectInfo rp(cam);
			if (!rp.GetPointCloudPlane() || rp.planeArray.size() < planeNum)
			{
				str_time = CTime::GetCurrentTime().Format("%X：");
			//	GetDlgItem(IDC_TEXTWORK)->SetWindowText(str_time+ error_txt+_T("\r\n"));
				continue;
			}
			else
			{
				str_time = CTime::GetCurrentTime().Format("%X：");
				work_txt += str_time + _T("粗识别结束\r\n");
			//	GetDlgItem(IDC_TEXTWORK)->SetWindowText(work_txt);
				rp.TwiceFitPlane();
				//
				str_time = CTime::GetCurrentTime().Format("%X：");
				work_txt += str_time + _T("精识别结束\r\n");
			//	GetDlgItem(IDC_TEXTWORK)->SetWindowText(work_txt);
				//rp.TabPlane();
				if (rp.IdentificationLine())
				{
					//rp.TabPlane();
					str_time = CTime::GetCurrentTime().Format("%X：");
					CString result;
					CString str1, str2, str3;
					str1.Format(_T("%f"), rp.line_len[0]);
					str2.Format(_T("%f"), rp.line_len[1]);
					str3.Format(_T("%f"), rp.line_len[2]);
					result = str_time + _T("边长:") + str1 + L"m " + str2 + L"m " + str3 + L"m \n";
					//work_txt += result;
					GetDlgItem(IDC_TEXTWORK)->SetWindowText(result);
					is_result = true;
				}
				else
				{
				//	str_time = CTime::GetCurrentTime().Format("%X：");
				//	GetDlgItem(IDC_TEXTWORK)->SetWindowText(str_time+error_txt+_T("\r\n"));
					continue;
				}
			}
		}
		imshow("opencv", cam.color_mat);
		cam.pc.clear();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CMy3DmearsureDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CMy3DmearsureDlg::OnBnClickedButstart()
{
	// TODO: 在此添加控件通知处理程序代码
	GetDlgItem(ID_BUTSTART)->SetWindowText(_T("继续"));
	Invalidate(false);
	is_start = true;
}

void CMy3DmearsureDlg::OnBnClickedButopen()
{
	// TODO: 在此添加控件通知处理程序代码
}


void CMy3DmearsureDlg::OnBnClickedOk2()
{
	// TODO: 在此添加控件通知处理程序代码
	
	destroyWindow("opencv");
	DestroyWindow();
}
