Dim bChangeIcon
bChangeIcon=False 
Dim bToT1
bToT1=True  

Set fso=CreateObject("Scripting.filesystemobject")  

SrcFile = createobject("Scripting.FileSystemObject").GetFile(Wscript.ScriptFullName).ParentFolder.Path&"\firmware.bin"  
DstFile = "F:\user\firmware.bin"  
If fso.FileExists(DstFile) Then fso.DeleteFile(DstFile) 
fso.CopyFile SrcFile,DstFile,True  

WScript.Sleep 1000

msgbox "Done!"
