Dim Version
Version="TL-D3V2.0.8.045.6"
Set fso=CreateObject("Scripting.filesystemobject")  
SrcFile = createobject("Scripting.FileSystemObject").GetFile(Wscript.ScriptFullName).ParentFolder.Path&"\firmware.bin"  
DstFile = "E:\user\"&Version&".bin"  
DstFile1 = createobject("Scripting.FileSystemObject").GetFile(Wscript.ScriptFullName).ParentFolder.Path&"\bin\1.bin"
DstFile2 = createobject("Scripting.FileSystemObject").GetFile(Wscript.ScriptFullName).ParentFolder.Path&"\"&Version&".bin"
If fso.FileExists(DstFile) Then fso.DeleteFile(DstFile)
If fso.FileExists(DstFile1) Then fso.DeleteFile(DstFile1)
If fso.FileExists(DstFile2) Then fso.DeleteFile(DstFile2)
fso.CopyFile SrcFile,DstFile1,True
fso.CopyFile SrcFile,DstFile2,True
fso.CopyFile SrcFile,DstFile,True  

WScript.Sleep 1000

msgbox "Done!"