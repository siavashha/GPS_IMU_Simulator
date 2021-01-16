using namespace System;
using namespace System::IO;
using namespace System::Collections;
using namespace std;
ref class CFileManager
{
	private:
		String ^ m_rootDirectory;
		IEnumerator ^ files;
	public:
		String ^ m_fileName;
		CFileManager(String^ );
		~CFileManager();
		bool GetNextFile();
		bool LoadDirFiles();
};