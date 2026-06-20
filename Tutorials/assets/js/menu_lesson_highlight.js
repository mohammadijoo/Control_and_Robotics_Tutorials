// menu_lesson_highlight.js
document.addEventListener("DOMContentLoaded", function () {
  const currentLesson = document.querySelector("#sidebar a.active-lesson");
  if (!currentLesson) return;

  // Find the corresponding chapter header (<span class="opener">)
  const lessonList = currentLesson.closest("ul");
  const chapterOpener = lessonList ? lessonList.previousElementSibling : null;

  // Expand the chapter using the menu's native toggle class (commonly "active")
  // This should NOT prevent toggling; it just sets the initial state to open.
  if (chapterOpener && chapterOpener.classList.contains("opener")) {
    chapterOpener.classList.add("active");
    chapterOpener.setAttribute("aria-expanded", "true"); // harmless if unused
  }

  // Scroll so the *lesson* is centered inside the sidebar (not the chapter header)
  const sidebar = document.querySelector("#sidebar .inner");
  if (!sidebar) {
    // Fallback: center in nearest scroll container (may also move the page)
    currentLesson.scrollIntoView({ block: "center", inline: "nearest", behavior: "smooth" });
    return;
  }

  const sidebarRect = sidebar.getBoundingClientRect();
  const lessonRect = currentLesson.getBoundingClientRect();

  // Lesson position relative to the sidebar scroll box
  const lessonOffsetTop = lessonRect.top - sidebarRect.top;

  // Compute target scrollTop so lesson ends up centered
  const target =
    sidebar.scrollTop +
    lessonOffsetTop -
    (sidebar.clientHeight / 2 - lessonRect.height / 2);

  // Clamp to valid scroll range
  const max = sidebar.scrollHeight - sidebar.clientHeight;
  const clamped = Math.max(0, Math.min(max, target));

  sidebar.scrollTo({
    top: clamped,
    behavior: "smooth",
  });
});
