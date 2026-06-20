document.querySelectorAll('pre code').forEach(codeBlock => {
  const pre = codeBlock.parentElement;

  // Skip if already added
  if (pre.querySelector('.copy-btn')) return;

  // Create and style the button
  const button = document.createElement('button');
  button.className = 'copy-btn';
  button.type = 'button';
  button.innerText = '📋 Copy';

  // Handle click to copy
  button.addEventListener('click', () => {
    const code = codeBlock.innerText;
    navigator.clipboard.writeText(code).then(() => {
      button.innerText = '✅ Copied';
      setTimeout(() => button.innerText = '📋 Copy', 1500);
    }).catch(() => {
      button.innerText = '❌ Failed';
      setTimeout(() => button.innerText = '📋 Copy', 1500);
    });
  });

  // Inject into <pre> so it’s above code but inside the dark box
  pre.appendChild(button);
});