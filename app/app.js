

let messageIndex = 0;

function showMessage(message, kind = "info") {
    let aname = "message" + messageIndex;
    messageIndex = messageIndex + 1;
    $('#alertsSpace').append(
        '<div id="' + aname + '" class="alert alert-' + kind + '"><a class="close" data-dismiss="alert">×</a><span>'+message+'</span></div>' 
    )
    setTimeout(function() {
        $('#' + aname).remove()
    }, 5000);
}

function showError(message) {
    showMessage(message, "danger");
}

function showWarning(message) {
    showMessage(message, "wanring")
}

function showSuccess(message) {
    showMessage(message, "success");
}








//showMessage("script loaded");



function mp3Upload() {
    console.log(this.files);
    f = this.files[0]
    console.log(f.size);
    let reader = new FileReader();
    reader.onload = mp3Convert;
    reader.readAsArrayBuffer(f);    
}

function mp3Convert(buffer) {
    console.log("Loaded " + buffer.byteLength());
}

function pageLoaded() {
    document.getElementById('mp3-uploader').addEventListener('change', mp3Upload, false)
    showMessage("attached")
}

$( document ).ready(function() {
    $('#mp3-uploader').change(mp3Upload)
    console.log( "ready!" );
});

