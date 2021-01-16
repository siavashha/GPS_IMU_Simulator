#include "CFileManager.h"

CFileManager::CFileManager(String ^rootDirectory)
{
	m_rootDirectory=rootDirectory;
	files = nullptr;
	m_fileName="";
}
CFileManager::~CFileManager()
{
	delete m_rootDirectory;
	delete files;
	delete m_fileName;
}
bool CFileManager::LoadDirFiles()
{
	if (Directory::Exists(m_rootDirectory))
		{
		array<String^>^ fileEntries = Directory::GetFiles(m_rootDirectory);
		files = fileEntries->GetEnumerator();
		}
	else
		{
		Console::WriteLine("The Directory has not been found.");
		}
	return true;
}

bool CFileManager::GetNextFile()
{
	static unsigned int count =0;
	bool nextFile=false;
	if (files->MoveNext())
		{
		nextFile=true;
		count++;
		Console::WriteLine("Count = " + count);
		m_fileName = safe_cast<String^> (files->Current);
		}
	return nextFile;
}


	