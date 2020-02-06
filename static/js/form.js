window.onload = function () {
$.getJSON("/graphs", function(data) {
    $('#new').text(data);
});
}