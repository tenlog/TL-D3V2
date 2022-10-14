Set fso=CreateObject("Scripting.filesystemobject")  
SrcFile = createobject("Scripting.FileSystemObject").GetFile(Wscript.ScriptFullName).ParentFolder.Path&"\firmware.bin"  
DstFile = "F:\user\firmware.bin"  
DstFile1 = createobject("Scripting.FileSystemObject").GetFile(Wscript.ScriptFullName).ParentFolder.Path&"\bin\1.bin"
If fso.FileExists(DstFile) Then fso.DeleteFile(DstFile) 
If fso.FileExists(DstFile1) Then fso.DeleteFile(DstFile1) 
fso.CopyFile SrcFile,DstFile1,True  
fso.CopyFile SrcFile,DstFile,True  

WScript.Sleep 1000

msgbox "Done!"
