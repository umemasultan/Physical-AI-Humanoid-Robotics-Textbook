# Deployment Guide - Physical AI Textbook

## 🚀 Quick Deploy to Vercel

### Prerequisites
- GitHub account
- Vercel account (free tier works)

### Step-by-Step Deployment

#### 1. Push to GitHub
```bash
git add .
git commit -m "Ready for deployment"
git push origin main
```

#### 2. Deploy on Vercel
1. Go to [vercel.com](https://vercel.com)
2. Click "Import Project"
3. Select your GitHub repository
4. Vercel will auto-detect settings from `vercel.json`
5. Click "Deploy"

#### 3. Configure Environment Variables (Optional)
If you have a backend deployed:
1. Go to Project Settings → Environment Variables
2. Add: `REACT_APP_BACKEND_URL` = `https://your-backend-url.com`
3. Redeploy

---

## 📱 What Works After Deployment

### ✅ Working Features:
- Complete textbook website
- All documentation pages
- Chatbot UI (button, chat window, voice input)
- Fallback chatbot responses (local knowledge base)
- Responsive design
- Search functionality

### ⚠️ Limited Without Backend:
- Chatbot uses pre-programmed responses
- No real-time RAG (Retrieval Augmented Generation)
- No dynamic content from vector database

---

## 🔧 Backend Deployment (Optional)

To enable full chatbot functionality, deploy the backend separately:

### Option 1: Render.com (Recommended - Free Tier)

1. Create account on [render.com](https://render.com)
2. Click "New +" → "Web Service"
3. Connect your GitHub repository
4. Configure:
   - **Name**: `physical-ai-backend`
   - **Root Directory**: `backend`
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
   - **Environment**: Python 3.12

5. Add Environment Variables:
   ```
   GROQ_API_KEY=your_groq_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_key
   CORS_ORIGINS=https://your-vercel-app.vercel.app
   ```

6. Click "Create Web Service"

7. Copy your backend URL (e.g., `https://physical-ai-backend.onrender.com`)

8. Update Vercel environment variable:
   - Go to Vercel Project Settings
   - Add: `REACT_APP_BACKEND_URL` = `https://physical-ai-backend.onrender.com`
   - Redeploy

### Option 2: Railway.app

1. Go to [railway.app](https://railway.app)
2. Click "New Project" → "Deploy from GitHub"
3. Select repository and `backend` folder
4. Add environment variables (same as above)
5. Deploy

---

## 🧪 Local Testing Before Deployment

### Test Production Build:
```bash
cd frontend
npm run build
npm run serve
```

Visit `http://localhost:3000` to test the production build.

### Test Chatbot Fallback:
1. Open the website
2. Click chatbot button
3. Try these queries:
   - "What is ROS 2?"
   - "Explain Digital Twin"
   - "Hello"
   - "Thanks"

All should work with fallback responses.

---

## 📝 Deployment Checklist

- [✓] API URL configured with environment variable support
- [✓] `.env.example` file created
- [✓] `.gitignore` includes `.env` files
- [✓] `vercel.json` optimized for Docusaurus
- [✓] Build succeeds locally (`npm run build`)
- [✓] No TypeScript errors (`npm run typecheck`)
- [✓] Fallback responses working in chatbot
- [✓] Cache headers configured for static assets

---

## 🔍 Troubleshooting

### Build fails on Vercel:
- Check Node.js version (should be >=18.0)
- Verify all dependencies in `package.json`
- Check build logs for specific errors

### Chatbot not working:
- Check browser console for errors
- Verify fallback responses are working
- If using backend, check CORS settings

### 404 errors on routes:
- Verify `vercel.json` has rewrites configured
- Check `docusaurus.config.js` baseUrl setting

---

## 🎯 Next Steps After Deployment

1. **Test all pages** - Navigate through all modules
2. **Test chatbot** - Try various questions
3. **Share the link** - Get feedback from users
4. **Monitor performance** - Check Vercel Analytics
5. **Deploy backend** (optional) - For full RAG functionality

---

## 📞 Support

Created by **Umema Sultan**

Issues? Check:
- GitHub repository
- Vercel deployment logs
- Browser console errors

---

## 🎉 Success!

Your Physical AI Textbook is now live! 🚀

Frontend URL: `https://your-project.vercel.app`
Backend URL (if deployed): `https://your-backend.onrender.com`
