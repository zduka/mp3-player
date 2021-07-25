
/* Shows messages at the top of the page. 
   
   Nothing fancy, the messages can be dismissed by the user, or will be deleted automatically after 5 seconds. 
 */
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


/** Loads and optionally converts the mp3 file to be added to a playlist.
 */
function mp3Upload() {
    console.log(this.files);
    f = this.files[0]
    console.log(f.size);
    let reader = new FileReader();
    reader.onload = function() {
        let buffer = reader.result;
        console.log("Loaded " + buffer.byteLength);
    };
    reader.readAsArrayBuffer(f);    
}

$( document ).ready(function() {
    $('#mp3-uploader').change(mp3Upload)
    console.log( "ready!" );
});

